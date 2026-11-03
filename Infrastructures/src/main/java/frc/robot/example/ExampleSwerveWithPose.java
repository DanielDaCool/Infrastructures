package frc.robot.example;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics.DemaciaKinematics;
import frc.robot.Kinematics.KinematicsConfig;
import frc.robot.Kinematics.SwerveAccelerationLimiter;
import frc.robot.Path.FollowPathCommand;
import frc.robot.Path.Path;
import frc.robot.Path.PathFollowerConfig;
import frc.robot.Path.PathDrivetrain;
import frc.robot.Path.PathsConfig;
import frc.robot.Path.Point;
import frc.robot.RobotPose.RobotPose;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Estimation.PhoenixOdometryThread;
import frc.robot.RobotPose.Vision.VisionConfig;
import frc.robot.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;
import frc.robot.RobotPose.Vision.visionConfigs.QuestConfig;

/**
 * Example swerve chassis that uses {@link DemaciaKinematics} (with {@link SwerveAccelerationLimiter})
 * and {@link RobotPose} together, with 250 Hz odometry from a {@link PhoenixOdometryThread}. Not used
 * by the robot: copy the parts you need into your own chassis. The modules are the robot's own code
 * ({@link Module}); the gyro is a Pigeon 2. See {@link ExampleChassis} for the Kinematics-only version.
 *
 * <p>Wiring, in {@code Robot}'s constructor:
 * <pre>
 * chassis = new ExampleSwerveWithPose(canBus, pigeon, modules, WHEEL_CIRCUMFERENCE, KINEMATICS_CONFIG, MODULE_LOCATIONS);
 * chassis.initializeRobotPose(ExampleSwerveWithPose.VisionConstants.VISION); // once, after the chassis
 * </pre>
 * and keep {@code Robot.robotPeriodic()} as it is: {@code CommandScheduler.run()}, then
 * {@code RobotPose.getInstance().periodic()}.
 *
 * <p>Loop order: {@link #periodic} (re-sync), then the commands (drive), then
 * {@code RobotPose.periodic()} adds this loop's odometry samples and vision. So a command reading
 * {@code getEstimatedPose()} gets the pose from the end of the previous loop, which already holds
 * every sample up to then; the drive heading is the gyro read in this loop (see {@link #heading()}).
 *
 * <p>Units and frames: meters, m/s, radians, FPGA seconds. Robot frame x forward, y left; field
 * frame blue-alliance origin (RobotPose doesn't flip for red, {@link #driveFromDriver} does). Everything
 * is CCW positive except the omega passed to {@code toSwerveModuleStates}, which {@link #drive} flips.
 *
 * <p>Module order: one array ({@code moduleLocations}) is passed to both {@link DemaciaKinematics}
 * and {@link RobotPose#initialize}, and the module states, the odometry thread's positions and the
 * modules all use the same index. A different order makes the robot fight itself when it turns
 * (the pose won't show it while driving straight).
 */
public class ExampleSwerveWithPose extends SubsystemBase implements PathDrivetrain {
    /** The cameras (same as the RobotPose README). Measure the offsets on your robot. */
    public static final class VisionConstants {
        // Limelight hostname: "limelight-front". Robot center (on the floor) to lens [m, rad]: x forward, y left, z up
        public static final LimelightTagCamera3dConfig FRONT = new LimelightTagCamera3dConfig(
                "front",
                new Transform3d(0.30, 0, 0.35, new Rotation3d(0, Math.toRadians(15), 0)),
                VecBuilder.fill(0.02, 0.02, Double.POSITIVE_INFINITY)); // unused by Limelights (distance based)

        // heading std infinite: vision never changes the heading, so the gyro stays the field heading
        public static final QuestConfig QUEST = new QuestConfig(
                "quest",
                new Transform3d(0.10, 0, 0.50, new Rotation3d()),
                VecBuilder.fill(0.05, 0.05, Double.POSITIVE_INFINITY));

        public static final VisionConfig VISION = new VisionConfig(FRONT, QUEST);

        private VisionConstants() {
        }
    }

    /** One swerve module: the robot's own module code (motors, encoders, PID). */
    public interface Module {
        /** Measured state: wheel speed [m/s] and module angle. */
        SwerveModuleState getState();

        /**
         * Drives the module. The state is already reversed and cosine scaled: don't optimize it again.
         *
         * @param state             wanted state
         * @param driveAcceleration drive acceleration feed-forward [m/s^2]
         * @param steerVelocity     steer velocity feed-forward [rad/s], CCW positive
         */
        void setState(SwerveModuleState state, double driveAcceleration, double steerVelocity);

        /** Drive motor position signal, in wheel rotations ({@code SensorToMechanismRatio} set). */
        StatusSignal<Angle> getDrivePosition();

        /** Steer position signal, in module rotations, CCW positive (fused CANcoder). */
        StatusSignal<Angle> getSteerPosition();
    }

    /** How much to trust the wheels (x m, y m, heading rad). Heading 0: vision never turns the robot. */
    private static final double ODOMETRY_STD_XY = 0.05;

    private final Module[] modules;
    private final Translation2d[] moduleLocations;
    private final DemaciaKinematics kinematics;
    private final SwerveAccelerationLimiter limiter;
    private final KinematicsConfig kinematicsConfig;
    private final PathsConfig pathsConfig;
    private final PathFollowerConfig followerConfig = PathFollowerConfig.defaults(); // gains, tolerances, red flip (ROTATE)
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw; // main thread copy (refreshed here, not by the thread) [deg]
    private final StatusSignal<AngularVelocity> yawRate; // [deg/s]
    private final PhoenixOdometryThread odometryThread;
    private ChassisSpeeds lastFieldSpeeds = new ChassisSpeeds(); // field-relative speeds really sent last (omega unused) [m/s, rad/s]
    private Rotation2d lastHeading = Rotation2d.kZero; // last finite gyro yaw
    private double yawWriteTime = Double.NEGATIVE_INFINITY; // Phoenix time the last setYaw returned [s]
    private boolean limitedLastLoop = false; // driveFieldRelative ran since the last periodic

    /**
     * Creates the kinematics and starts the odometry thread. Call {@link #initializeRobotPose} after.
     *
     * @param canBus             the bus the Pigeon and the module motors are on (all on one bus)
     * @param pigeon             the gyro, CCW positive
     * @param modules            the modules, in the same order as the locations
     * @param wheelCircumference wheel circumference [m]
     * @param kinematicsConfig   kinematics and acceleration limiter settings for this robot
     * @param moduleLocations    module positions on the robot [m], x forward, y left
     */
    public ExampleSwerveWithPose(CANBus canBus, Pigeon2 pigeon, Module[] modules, double wheelCircumference,
            KinematicsConfig kinematicsConfig, Translation2d... moduleLocations) {
        if (modules.length != moduleLocations.length) {
            throw new IllegalArgumentException("one location per module, in the same order");
        }
        this.modules = modules;
        this.moduleLocations = moduleLocations;
        this.kinematicsConfig = kinematicsConfig;
        this.pathsConfig = PathsConfig.defaults(); // omega and alpha limits: placeholders, measure on the robot
        this.kinematics = new DemaciaKinematics(kinematicsConfig, moduleLocations);
        this.limiter = new SwerveAccelerationLimiter(kinematicsConfig);
        this.pigeon = pigeon;
        this.yaw = pigeon.getYaw();
        this.yawRate = pigeon.getAngularVelocityZWorld();

        // Clones for the thread, so the main thread refreshing the originals doesn't change them under it
        BaseStatusSignal threadYaw = pigeon.getYaw().clone();
        BaseStatusSignal pitch = pigeon.getPitch().clone();
        BaseStatusSignal roll = pigeon.getRoll().clone();
        BaseStatusSignal[] drive = new BaseStatusSignal[modules.length];
        BaseStatusSignal[] steer = new BaseStatusSignal[modules.length];
        List<BaseStatusSignal> signals = new ArrayList<>(List.of(threadYaw, pitch, roll));
        for (int i = 0; i < modules.length; i++) {
            drive[i] = modules[i].getDrivePosition().clone();
            steer[i] = modules[i].getSteerPosition().clone();
            signals.add(drive[i]);
            signals.add(steer[i]);
        }

        // 250 Hz on CAN FD, 100 Hz on CAN 2.0. Starts now. Call optimizeBusUtilization (if used) after this.
        odometryThread = new PhoenixOdometryThread(canBus, () -> {
            // Runs on the odometry thread: only read the values, never refresh or touch other devices
            SwerveModulePosition[] positions = new SwerveModulePosition[drive.length];
            for (int i = 0; i < drive.length; i++) {
                positions[i] = new SwerveModulePosition(
                        drive[i].getValueAsDouble() * wheelCircumference, // wheel rotations -> m
                        Rotation2d.fromRotations(steer[i].getValueAsDouble()));
            }
            double tilt = OdometryData.tilt(Math.toRadians(pitch.getValueAsDouble()),
                    Math.toRadians(roll.getValueAsDouble()));
            // time 0: the thread stamps it (FPGA seconds minus the CAN latency)
            return new OdometryData(0, Rotation2d.fromDegrees(threadYaw.getValueAsDouble()), positions, tilt);
        }, signals.toArray(new BaseStatusSignal[0]));
    }

    /**
     * Creates {@link RobotPose}. Call once, in {@code Robot}'s constructor, right after this chassis.
     *
     * <p>Both gyro setters are {@link #writeGyro}: {@code setYaw} and {@code resetPose} (and the
     * dashboard buttons "chassis/reset gyro" and "pose/reset pose") write the new heading to the
     * Pigeon. So the gyro always equals the pose heading, and the acceleration limit turns with it.
     */
    public void initializeRobotPose(VisionConfig vision) {
        RobotPose.initialize(
                odometryThread::getNewSamples, // every 250 Hz sample since the last loop
                this::writeGyro, // setYaw() writes the gyro
                this::writeGyro, // resetPose() writes the gyro too
                moduleLocations, // same order as the module positions
                VecBuilder.fill(ODOMETRY_STD_XY, ODOMETRY_STD_XY, 0),
                vision);
    }

    /**
     * Field heading for driving: the gyro, read in this loop.
     *
     * <p>Why the gyro and not {@code getEstimatedPose().getRotation()}: both are the same heading
     * here (both RobotPose setters write the gyro, and no vision source changes the heading, heading
     * std infinite / 0), but the pose is from the end of the last loop, about 20 ms old: at 4.5 rad/s
     * that is 5 degrees off. If you ever let vision change the heading, or pass no pose reset gyro
     * setter, use {@code RobotPose.getInstance().getEstimatedPose().getRotation()} instead.
     *
     * @return gyro yaw, CCW positive, or the last finite one when the gyro gives NaN or infinity
     */
    private Rotation2d heading() {
        yaw.refresh();
        if (yaw.getAllTimestamps().getSystemTimestamp().getTime() <= yawWriteTime) {
            return lastHeading; // frame received before the write's ack: still the old yaw
        }
        Rotation2d gyro = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        if (Double.isFinite(gyro.getCos()) && Double.isFinite(gyro.getSin())) {
            lastHeading = gyro;
        }
        return lastHeading;
    }

    /** Gyro yaw rate [rad/s], CCW positive. */
    private double getGyroRate() {
        yawRate.refresh();
        return Math.toRadians(yawRate.getValueAsDouble());
    }

    /**
     * Writes the yaw to the Pigeon. Only RobotPose calls it (through {@link #setYaw} and
     * {@link #setStartPose}), after it has added the samples measured before the reset.
     *
     * @param newYaw new yaw, CCW positive
     */
    private void writeGyro(Rotation2d newYaw) {
        Rotation2d turn = newYaw.minus(heading());
        pigeon.setYaw(newYaw.getDegrees()); // blocks until the ack (up to 100 ms)
        yawWriteTime = Utils.getCurrentTimeSeconds();
        lastHeading = newYaw; // the next yaw frame may still hold the old value; heading() skips it
        // the limiter's speeds are field-relative: the same motion seen from the new field angle
        Translation2d velocity = new Translation2d(lastFieldSpeeds.vxMetersPerSecond,
                lastFieldSpeeds.vyMetersPerSecond).rotateBy(turn);
        lastFieldSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), lastFieldSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Sets the heading only, keeping the position (gyro, pose, Quest and limiter together). Always
     * go through RobotPose: calling {@code pigeon.setYaw} directly would make the pose jump.
     *
     * @param newYaw new field heading (blue-alliance origin), CCW positive
     */
    public void setYaw(Rotation2d newYaw) {
        RobotPose.getInstance().setYaw(newYaw);
    }

    /** Driver "zero heading": the robot now faces away from the driver's alliance wall. */
    public void zeroHeading() {
        setYaw(isRedAlliance() ? Rotation2d.kPi : Rotation2d.kZero);
    }

    /**
     * Sets the whole pose, e.g. the auto start pose. Also writes the gyro (see {@link #initializeRobotPose}).
     *
     * @param pose field pose, blue-alliance origin (flip it yourself for red, e.g. the path follower's flip)
     */
    public void setStartPose(Pose2d pose) {
        RobotPose.getInstance().resetPose(pose);
    }

    /** The fused field pose (blue-alliance origin), as of the last {@code RobotPose.periodic()}. */
    @Override
    public Pose2d getPose() {
        return RobotPose.getInstance().getEstimatedPose();
    }

    private static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /** Measured module states, in the module order. */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Robot-relative drive, no acceleration limit. Call once per loop.
     *
     * @param robotSpeeds robot-relative speeds [m/s, rad/s], omega CCW positive
     */
    public void drive(ChassisSpeeds robotSpeeds) {
        // toSwerveModuleStates takes omega clockwise-positive
        ChassisSpeeds clockwise = new ChassisSpeeds(robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond, -robotSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(clockwise, getModuleStates());
        double[] driveAccelerations = kinematics.getDriveAccelerations();
        double[] steerVelocities = kinematics.getSteerVelocities();
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i], driveAccelerations[i], steerVelocities[i]);
        }
    }

    /**
     * Field-relative drive with the acceleration limit. Call once per loop, instead of {@link #drive}.
     *
     * @param wanted field-relative speeds (blue-alliance origin) [m/s, rad/s], omega CCW positive
     */
    @Override
    public void driveFieldRelative(ChassisSpeeds wanted) {
        Rotation2d heading = heading();
        // desaturate before the limit, or the robot spins at full speed and drives much slower
        wanted = ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.desaturate(ChassisSpeeds.fromFieldRelativeSpeeds(wanted, heading)), heading);
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(limiter.calculate(wanted, lastFieldSpeeds, heading), heading));
        // next loop starts from what was really sent (after cosine scaling), not from the limit's output
        lastFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.getSentSpeeds(), heading);
        limitedLastLoop = true;
    }

    /**
     * Teleop drive from the driver's point of view: x away from the driver's alliance wall, y to the
     * driver's left. On red that is the field's -x, so the speeds are turned 180 degrees.
     *
     * @param driverSpeeds driver-relative speeds [m/s, rad/s], omega CCW positive (not flipped)
     */
    public void driveFromDriver(ChassisSpeeds driverSpeeds) {
        double sign = isRedAlliance() ? -1 : 1;
        driveFieldRelative(new ChassisSpeeds(sign * driverSpeeds.vxMetersPerSecond,
                sign * driverSpeeds.vyMetersPerSecond, driverSpeeds.omegaRadiansPerSecond));
    }

    /**
     * Measured field-relative speeds: the module states (with the gyro rate for omega) turned by the
     * gyro heading. Same two lines as the re-sync in {@link #periodic}. For the path follower.
     *
     * @return measured speeds, blue-alliance origin [m/s, rad/s], omega CCW positive
     */
    @Override
    public ChassisSpeeds getFieldSpeeds() {
        ChassisSpeeds measured = kinematics.toChassisSpeeds(getModuleStates(), getGyroRate());
        return ChassisSpeeds.fromRobotRelativeSpeeds(measured, heading());
    }

    /** Example path (blue alliance, meters). Replace with your own points; see docs/Path.md. */
    public static Path examplePath() {
        return new Path(
                new Point(1, 2),
                new Point(3, 4).withVelocity(1.5), // slow to at most 1.5 m/s at this corner
                new Point(5, 2).stop(1.0))         // stop here for 1 s
                .withName("example");
    }

    /**
     * A fresh path-following command: on red it drives the flipped path, and the start pose is set
     * (flipped too) first. Call it again for every autonomous run: a command can be used in one
     * composition only, e.g. {@code chassis.follow(examplePath())}. Pass the blue path; the command flips it.
     *
     * <p>To run a mechanism part way along the path, keep the command and use its progress (0 to 1):
     * <pre>{@code
     * FollowPathCommand follow = chassis.followCommand(examplePath());
     * new Trigger(() -> follow.progress() > 0.7).onTrue(intake.runCommand());
     * }</pre>
     *
     * @param path the path, written for the blue alliance
     * @return a new command that requires this chassis
     */
    public Command follow(Path path) {
        return Commands.defer(() -> {
            Path active = isRedAlliance() ? path.flipped(followerConfig) : path; // red: the flipped path (start pose)
            Pose2d start = new Pose2d(active.startPoint(), active.startHeading().orElse(getPose().getRotation()));
            return Commands.runOnce(() -> setStartPose(start)).andThen(followCommand(path));
        }, Set.of(this));
    }

    /**
     * The follower alone (no start pose reset), with this robot's constraints. The command flips the
     * path itself on red.
     *
     * @param path the path, written for the blue alliance
     * @return a new command that requires this chassis
     */
    public FollowPathCommand followCommand(Path path) {
        return new FollowPathCommand(this, path).withConfig(followerConfig);
    }

    @Override
    public KinematicsConfig getKinematicsConfig() {
        return kinematicsConfig;
    }

    @Override
    public PathsConfig getPathsConfig() {
        return pathsConfig;
    }

    /**
     * Re-sync: when something else drove the robot last loop (just enabled, {@link #drive}, a path
     * follower), the limit starts from the measured speeds, not from what it sent long ago.
     * {@code periodic} runs before the commands, so the flag is still last loop's.
     */
    @Override
    public void periodic() {
        if (!limitedLastLoop) {
            lastFieldSpeeds = getFieldSpeeds();
        }
        limitedLastLoop = false;
    }
}
