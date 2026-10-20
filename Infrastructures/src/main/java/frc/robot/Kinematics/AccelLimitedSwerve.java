package frc.robot.Kinematics;

import static frc.robot.Kinematics.KinematicsConstants.MIN_VELOCITY;

import edu.wpi.first.math.geometry.Rotation2d;
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
 * <p>The sent speeds are kept field-relative (what the limiter works in). They are the speeds
 * the modules really got, after desaturation, read back with
 * {@link DemaciaKinematics#toChassisSpeeds(SwerveModuleState[])} (the exact inverse). They are not
 * kept robot-relative: then a robot turning while driving would see its last speeds turn with it
 * every loop, and the limiter could not keep up with a fast turn.
 */
public class AccelLimitedSwerve {

    private final DemaciaKinematics kinematics;
    private final SwerveAccelerationLimiter limiter = new SwerveAccelerationLimiter();
    private final Rotation2d[] lastAngles; // module angles kept when the robot stops
    private ChassisSpeeds sentSpeedsFieldRel = new ChassisSpeeds();
    private Rotation2d lastHeading = Rotation2d.kZero;

    public AccelLimitedSwerve(DemaciaKinematics kinematics) {
        this.kinematics = kinematics;
        lastAngles = new Rotation2d[kinematics.getModuleCount()];
        for (int i = 0; i < lastAngles.length; i++) {
            lastAngles[i] = Rotation2d.kZero;
        }
    }

    /**
     * Module states for this loop: the wanted speeds limited by the acceleration limiter and
     * desaturated. Call once per loop.
     *
     * @param wantedFieldRel wanted field-relative speeds [m/s, rad/s]
     * @param heading        robot heading (gyro angle), CCW positive
     * @return module states to send. When the robot should stand, every module gets speed 0 and
     *         keeps its last angle
     */
    public SwerveModuleState[] calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading) {
        return send(limiter.calculate(wantedFieldRel, sentSpeedsFieldRel, heading), heading);
    }

    /** Same as {@link #calculate(ChassisSpeeds, Rotation2d)} with a given dt [s], for tests. */
    SwerveModuleState[] calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading, double dt) {
        return send(SwerveAccelerationLimiter.calculate(wantedFieldRel, sentSpeedsFieldRel, heading, dt), heading);
    }

    private SwerveModuleState[] send(ChassisSpeeds limitedFieldRel, Rotation2d heading) {
        lastHeading = heading;
        ChassisSpeeds robotRel = ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldRel, heading);
        SwerveModuleState[] states = new SwerveModuleState[lastAngles.length];

        if (Math.hypot(robotRel.vxMetersPerSecond, robotRel.vyMetersPerSecond) < MIN_VELOCITY
                && Math.abs(robotRel.omegaRadiansPerSecond) < MIN_VELOCITY) {
            // standing: don't turn the modules back to 0
            for (int i = 0; i < states.length; i++) {
                states[i] = new SwerveModuleState(0, lastAngles[i]);
            }
            sentSpeedsFieldRel = new ChassisSpeeds();
            return states;
        }

        states = kinematics.toSwerveModuleStates(robotRel);
        for (int i = 0; i < states.length; i++) {
            lastAngles[i] = states[i].angle;
        }
        sentSpeedsFieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(states), heading);
        return states;
    }

    /**
     * Starts the next loop from the measured speeds instead of the sent ones. Call it when
     * something else drove the modules last loop.
     *
     * @param measuredFieldRel measured field-relative speeds
     */
    public void resetTo(ChassisSpeeds measuredFieldRel) {
        sentSpeedsFieldRel = measuredFieldRel;
    }

    /**
     * Same as {@link #resetTo(ChassisSpeeds)}, and the modules keep these angles when the robot
     * stands (e.g. the measured module states when the robot starts).
     */
    public void resetTo(ChassisSpeeds measuredFieldRel, SwerveModuleState[] measuredStates) {
        resetTo(measuredFieldRel);
        for (int i = 0; i < lastAngles.length; i++) {
            lastAngles[i] = measuredStates[i].angle;
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
        sentSpeedsFieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(sentSpeedsFieldRel, oldYaw), newYaw);
        lastHeading = newYaw;
    }

    /** The field-relative speeds sent last loop (after desaturation). */
    public ChassisSpeeds getSentSpeedsFieldRel() {
        return sentSpeedsFieldRel;
    }

    /** The robot-relative speeds sent last loop (after desaturation). */
    public ChassisSpeeds getSentSpeedsRobotRel() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(sentSpeedsFieldRel, lastHeading);
    }
}
