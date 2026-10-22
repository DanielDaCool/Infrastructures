package frc.robot.Kinematics;

import static frc.robot.Kinematics.KinematicsConstants.CYCLE_DT;
import static frc.robot.Kinematics.KinematicsConstants.MIN_VELOCITY;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Drives a swerve with the {@link SwerveAccelerationLimiter}: keeps the speeds sent last loop
 * and gives the module states for this loop.
 *
 * <p>Every loop: {@code setModuleStates(drive.calculate(wantedFieldRel, gyroAngle))}. When
 * something else drove the modules last loop (another command, disabled), call
 * {@link #resetTo} with the measured speeds first, so the limiter starts from what the robot is
 * really doing. When the gyro yaw is reset, call {@link #onYawReset}.
 *
 * <p>The module states are discretized ({@code ChassisSpeeds.discretize(speeds, CYCLE_DT)}):
 * a state held for one loop while the robot turns at omega moves the robot along the wanted
 * direction turned by omega * dt / 2, so without it the robot drifts sideways while it spins.
 * The speeds are desaturated first, so the discretize is done on speeds the modules can really
 * drive. The discretize makes the translation slightly bigger, so at full speed the states are
 * desaturated again; that leaves a drift of about a millimeter per 2 s while spinning fast.
 *
 * <p>The sent speeds are kept field-relative (what the limiter works in). They are the speeds
 * the modules really got, after desaturation, read back with
 * {@link DemaciaKinematics#toChassisSpeeds(SwerveModuleState[])} (the exact inverse) and with the
 * discretize undone, so they are the mean velocity over the loop and not the module vector
 * turned by omega * dt / 2. They are not kept robot-relative: then a robot turning while driving
 * would see its last speeds turn with it every loop, and the limiter could not keep up with a
 * fast turn.
 *
 * <p>The steer-aware {@link #calculate(ChassisSpeeds, Rotation2d, double, Rotation2d[], double[])} adds
 * latency compensation, cosine scaling and the feed-forwards on top of this.
 *
 * <p>A heading that is NaN or infinite (gyro glitch) is replaced by the last heading. A gyro that
 * stays NaN freezes the heading.
 */
public class AccelLimitedSwerve {

    private final DemaciaKinematics kinematics;
    private final SwerveAccelerationLimiter limiter = new SwerveAccelerationLimiter();
    private final Rotation2d[] lastAngles; // module angles kept when the robot stops
    private ChassisSpeeds sentSpeedsFieldRel = new ChassisSpeeds();
    private Rotation2d lastHeading = Rotation2d.kZero;
    private final ModuleCommandConfig config;
    private final ModuleCommand[] previousCommands; // for the steer-aware calculate
    private final boolean[] reversed;

    public AccelLimitedSwerve(DemaciaKinematics kinematics) {
        this(kinematics, ModuleCommandConfig.defaults());
    }

    /**
     * @param config used by the steer-aware {@link #calculate(ChassisSpeeds, Rotation2d, double,
     *               Rotation2d[], double[])}
     */
    public AccelLimitedSwerve(DemaciaKinematics kinematics, ModuleCommandConfig config) {
        this.kinematics = kinematics;
        this.config = Objects.requireNonNull(config);
        lastAngles = new Rotation2d[kinematics.getModuleCount()];
        previousCommands = new ModuleCommand[lastAngles.length];
        reversed = new boolean[lastAngles.length];
        for (int i = 0; i < lastAngles.length; i++) {
            lastAngles[i] = Rotation2d.kZero;
            previousCommands[i] = new ModuleCommand(Rotation2d.kZero, 0, 0, 0);
        }
    }

    /**
     * Module states for this loop: the wanted speeds limited by the acceleration limiter,
     * desaturated and discretized. Call once per loop.
     *
     * @param wantedFieldRel wanted field-relative speeds [m/s, rad/s]
     * @param heading        robot heading (gyro angle), CCW positive. NaN or infinite uses the
     *                       last heading
     * @return module states to send. When the robot should stand, every module gets speed 0 and
     *         keeps its last angle
     */
    public SwerveModuleState[] calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading) {
        heading = finiteOrLast(heading);
        return send(limiter.calculate(wantedFieldRel, sentSpeedsFieldRel, heading), heading);
    }

    /** Same as {@link #calculate(ChassisSpeeds, Rotation2d)} with a given dt [s], for tests. */
    SwerveModuleState[] calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading, double dt) {
        heading = finiteOrLast(heading);
        return send(SwerveAccelerationLimiter.calculate(wantedFieldRel, sentSpeedsFieldRel, heading, dt), heading);
    }

    /**
     * Module commands for this loop, steer-aware: latency compensation, the same limiting and
     * discretizing as {@link #calculate(ChassisSpeeds, Rotation2d)}, then
     * {@link DemaciaKinematics#toModuleCommands} (reverse hysteresis, cosine scaling, feed-forwards).
     * Keeps the previous commands and reversed flags. Call once per loop, and not mixed with the
     * other {@code calculate} in the same loop.
     *
     * <p>The heading used is {@code heading + gyroRate * latencySeconds} (config), also for the
     * speeds kept for the limiter.
     *
     * @param wantedFieldRel          wanted field-relative speeds
     * @param heading                 gyro heading, CCW positive. NaN or infinite uses the last heading
     * @param gyroRateRadiansPerSecond gyro yaw rate [rad/s]. NaN or infinite counts as 0
     * @param measuredAngles          measured module angles
     * @param measuredSteerVelocities measured steer velocities [rad/s]
     * @return one command per module. Send speed + acceleration to the drive, angle + steer
     *         velocity to the steer. When the robot should stand, every module keeps its angle,
     *         speed 0 and a braking acceleration
     * @throws IllegalArgumentException when an array does not have one entry per module
     */
    public ModuleCommand[] calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading,
            double gyroRateRadiansPerSecond, Rotation2d[] measuredAngles, double[] measuredSteerVelocities) {
        checkLengths(measuredAngles, measuredSteerVelocities);
        heading = compensated(finiteOrLast(heading), gyroRateRadiansPerSecond);
        return sendCommands(limiter.calculate(wantedFieldRel, sentSpeedsFieldRel, heading), heading,
                measuredAngles, measuredSteerVelocities, CYCLE_DT);
    }

    /** Same as the steer-aware calculate with a given dt [s], for tests. */
    ModuleCommand[] calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading, double gyroRateRadiansPerSecond,
            Rotation2d[] measuredAngles, double[] measuredSteerVelocities, double dt) {
        checkLengths(measuredAngles, measuredSteerVelocities);
        heading = compensated(finiteOrLast(heading), gyroRateRadiansPerSecond);
        return sendCommands(SwerveAccelerationLimiter.calculate(wantedFieldRel, sentSpeedsFieldRel, heading, dt),
                heading, measuredAngles, measuredSteerVelocities, dt);
    }

    private void checkLengths(Rotation2d[] measuredAngles, double[] measuredSteerVelocities) {
        if (measuredAngles.length != lastAngles.length || measuredSteerVelocities.length != lastAngles.length) {
            throw new IllegalArgumentException("measured arrays need one entry per module (" + lastAngles.length + ")");
        }
    }

    /** The heading advanced by the gyro rate times the latency (a NaN or infinite rate counts as 0). */
    private Rotation2d compensated(Rotation2d heading, double gyroRate) {
        double rate = Double.isFinite(gyroRate) ? gyroRate : 0;
        return heading.plus(new Rotation2d(rate * config.latencySeconds()));
    }

    private ModuleCommand[] sendCommands(ChassisSpeeds limitedFieldRel, Rotation2d heading,
            Rotation2d[] measuredAngles, double[] measuredSteerVelocities, double dt) {
        lastHeading = heading;
        ChassisSpeeds robotRel = ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldRel, heading);
        ModuleCommand[] commands;
        if (isStanding(robotRel)) {
            commands = DemaciaKinematics.toStandingCommands(previousCommands, config, dt);
            sentSpeedsFieldRel = new ChassisSpeeds();
        } else {
            ChassisSpeeds discretized = desaturateAndDiscretize(robotRel);
            // the speeds sent are read back from the planned states, before the cosine scaling
            readBack(kinematics.toSwerveModuleStates(discretized), heading);
            commands = kinematics.toModuleCommands(discretized, measuredAngles, measuredSteerVelocities,
                    previousCommands, reversed, config, dt);
        }
        for (int i = 0; i < commands.length; i++) {
            previousCommands[i] = commands[i];
            lastAngles[i] = commands[i].angle();
        }
        return commands;
    }

    private static boolean isStanding(ChassisSpeeds robotRel) {
        return Math.hypot(robotRel.vxMetersPerSecond, robotRel.vyMetersPerSecond) < MIN_VELOCITY
                && Math.abs(robotRel.omegaRadiansPerSecond) < MIN_VELOCITY;
    }

    /** Desaturate first, then discretize the speeds the modules can really drive. */
    private ChassisSpeeds desaturateAndDiscretize(ChassisSpeeds robotRel) {
        ChassisSpeeds desaturated = kinematics.toChassisSpeeds(kinematics.toSwerveModuleStates(robotRel));
        return ChassisSpeeds.discretize(desaturated, CYCLE_DT);
    }

    /** The heading, or the last heading when it is NaN or infinite (gyro glitch). */
    private Rotation2d finiteOrLast(Rotation2d heading) {
        return Double.isFinite(heading.getCos()) && Double.isFinite(heading.getSin()) ? heading : lastHeading;
    }

    private SwerveModuleState[] send(ChassisSpeeds limitedFieldRel, Rotation2d heading) {
        lastHeading = heading;
        ChassisSpeeds robotRel = ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldRel, heading);
        SwerveModuleState[] states = new SwerveModuleState[lastAngles.length];

        if (isStanding(robotRel)) {
            // standing: don't turn the modules back to 0
            for (int i = 0; i < states.length; i++) {
                states[i] = new SwerveModuleState(0, lastAngles[i]);
            }
            sentSpeedsFieldRel = new ChassisSpeeds();
            return states;
        }

        states = kinematics.toSwerveModuleStates(desaturateAndDiscretize(robotRel));
        for (int i = 0; i < states.length; i++) {
            lastAngles[i] = states[i].angle;
        }
        readBack(states, heading);
        return states;
    }

    /**
     * Keeps the speeds the states ask for, with the discretize undone: the states move the robot
     * along an arc, its chord over one loop is the mean velocity.
     */
    private void readBack(SwerveModuleState[] states, Rotation2d heading) {
        ChassisSpeeds d = kinematics.toChassisSpeeds(states);
        Pose2d moved = Pose2d.kZero.exp(new Twist2d(d.vxMetersPerSecond * CYCLE_DT,
                d.vyMetersPerSecond * CYCLE_DT, d.omegaRadiansPerSecond * CYCLE_DT));
        ChassisSpeeds sentRobotRel = new ChassisSpeeds(moved.getX() / CYCLE_DT, moved.getY() / CYCLE_DT,
                d.omegaRadiansPerSecond);
        sentSpeedsFieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(sentRobotRel, heading);
    }

    /**
     * Starts the next loop from the measured speeds instead of the sent ones. Call it when
     * something else drove the modules last loop.
     *
     * <p>Speeds read from measured module states are the module vectors, not the mean velocity
     * over the loop, so while the robot turns their direction is about omega * dt / 2 off (small).
     *
     * @param measuredFieldRel measured field-relative speeds
     */
    public void resetTo(ChassisSpeeds measuredFieldRel) {
        sentSpeedsFieldRel = measuredFieldRel;
    }

    /**
     * Same as {@link #resetTo(ChassisSpeeds)}, and the modules keep these angles when the robot
     * stands (e.g. the measured module states when the robot starts). For the steer-aware
     * calculate it also resets the previous commands (measured angle and speed) and the reversed
     * flags (speed below 0).
     */
    public void resetTo(ChassisSpeeds measuredFieldRel, SwerveModuleState[] measuredStates) {
        resetTo(measuredFieldRel);
        for (int i = 0; i < lastAngles.length; i++) {
            lastAngles[i] = measuredStates[i].angle;
            // the steer-aware calculate starts from the measured angles and speeds too
            previousCommands[i] = new ModuleCommand(measuredStates[i].angle, measuredStates[i].speedMetersPerSecond,
                    0, 0);
            reversed[i] = measuredStates[i].speedMetersPerSecond < 0;
        }
    }

    /**
     * Call when the gyro yaw is set to a new value. The robot didn't move, so its robot-relative
     * speeds stay the same, but the field-relative ones are now measured from another angle.
     *
     * @param oldYaw the yaw before the reset
     * @param newYaw the yaw after the reset
     */
    public void onYawReset(Rotation2d oldYaw, Rotation2d newYaw) {
        oldYaw = finiteOrLast(oldYaw);
        newYaw = finiteOrLast(newYaw);
        sentSpeedsFieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(sentSpeedsFieldRel, oldYaw), newYaw);
        lastHeading = newYaw;
    }

    /**
     * The field-relative speeds sent last loop (after desaturation), with the discretize undone:
     * the mean velocity of the robot over the loop.
     */
    public ChassisSpeeds getSentSpeedsFieldRel() {
        return sentSpeedsFieldRel;
    }

    /**
     * The robot-relative speeds sent last loop (after desaturation, discretize undone), with the
     * heading of that loop.
     */
    public ChassisSpeeds getSentSpeedsRobotRel() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(sentSpeedsFieldRel, lastHeading);
    }
}
