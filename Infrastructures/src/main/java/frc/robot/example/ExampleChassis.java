package frc.robot.example;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics.DemaciaKinematics;
import frc.robot.Kinematics.SwerveAccelerationLimiter;

/**
 * Example swerve chassis that uses {@link DemaciaKinematics}, with the optional acceleration limit
 * ({@link SwerveAccelerationLimiter}). Not used by the robot: copy the parts you need into your own
 * chassis. The hardware is the robot's own code ({@link Module} and the gyro methods).
 *
 * <p>Speeds and the gyro are in the WPILib frame (x forward, y left, CCW positive). Only
 * {@code toSwerveModuleStates} takes omega clockwise-positive; {@link #drive} flips it.
 *
 * <p>The acceleration limit is three parts: {@link #driveFieldRelative} (once per loop), the
 * re-sync in {@link #periodic} and {@link #setYaw} for every gyro reset.
 */
public abstract class ExampleChassis extends SubsystemBase {
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
    }

    private final Module[] modules;
    private final DemaciaKinematics kinematics;
    private final SwerveAccelerationLimiter limiter = new SwerveAccelerationLimiter();
    private ChassisSpeeds lastFieldSpeeds = new ChassisSpeeds(); // field-relative speeds the limit sent last [m/s, rad/s]
    private boolean limitedLastLoop = false; // driveFieldRelative ran since the last periodic

    /**
     * @param modules         the modules, in the same order as the positions
     * @param modulePositions module positions on the robot [m], x forward, y left
     */
    protected ExampleChassis(Module[] modules, Translation2d... modulePositions) {
        this.modules = modules;
        this.kinematics = new DemaciaKinematics(modulePositions);
    }

    /** Gyro yaw, CCW positive. */
    protected abstract Rotation2d getGyroAngle();

    /** Gyro yaw rate [rad/s], CCW positive. */
    protected abstract double getGyroRate();

    /** Writes the yaw to the gyro hardware. */
    protected abstract void setGyroYaw(Rotation2d yaw);

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
     * @param wanted field-relative speeds [m/s, rad/s], omega CCW positive
     */
    public void driveFieldRelative(ChassisSpeeds wanted) {
        Rotation2d heading = getGyroAngle();
        // desaturate before the limit, or the robot spins at full speed and drives much slower
        wanted = ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.desaturate(ChassisSpeeds.fromFieldRelativeSpeeds(wanted, heading)), heading);
        lastFieldSpeeds = limiter.calculate(wanted, lastFieldSpeeds, heading);
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(lastFieldSpeeds, heading));
        limitedLastLoop = true;
    }

    /**
     * Re-sync: when something else drove the robot last loop (just enabled, {@link #drive}, a path
     * follower), the limit starts from the measured speeds, not from what it sent long ago.
     * {@code periodic} runs before the commands, so the flag is still last loop's.
     */
    @Override
    public void periodic() {
        if (!limitedLastLoop) {
            ChassisSpeeds measured = kinematics.toChassisSpeeds(getModuleStates(), getGyroRate());
            lastFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(measured, getGyroAngle());
        }
        limitedLastLoop = false;
    }

    /**
     * Sets the gyro yaw. Use it for every yaw reset (driver button, auto start, pose reset).
     *
     * @param newYaw new yaw, CCW positive
     */
    public void setYaw(Rotation2d newYaw) {
        Rotation2d turn = newYaw.minus(getGyroAngle());
        setGyroYaw(newYaw);
        // the speeds are field-relative: the same motion seen from the new field angle
        Translation2d velocity = new Translation2d(lastFieldSpeeds.vxMetersPerSecond,
                lastFieldSpeeds.vyMetersPerSecond).rotateBy(turn);
        lastFieldSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), lastFieldSpeeds.omegaRadiansPerSecond);
    }
}
