package frc.robot.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Kinematics.KinematicsConfig;

/**
 * What {@link FollowPathCommand} needs from a drivetrain. Implement it on your swerve subsystem:
 *
 * <pre>{@code
 * public class Swerve extends SubsystemBase implements PathDrivetrain {
 *     public Pose2d getPose() { return odometry.getEstimatedPosition(); }
 *     public ChassisSpeeds getFieldSpeeds() { return measuredFieldSpeeds(); }
 *     public void driveFieldRelative(ChassisSpeeds speeds) { setFieldSpeeds(speeds); }
 *     // optional: override getKinematicsConfig() / getPathsConfig() with your robot's limits
 * }
 * }</pre>
 */
public interface PathDrivetrain extends Subsystem {

    /** @return blue-origin field pose */
    Pose2d getPose();

    /** @return measured field-relative speeds, omega CCW */
    ChassisSpeeds getFieldSpeeds();

    /** @param speeds the field-relative command (blue frame), omega CCW */
    void driveFieldRelative(ChassisSpeeds speeds);

    /** @return the robot limits used when a path has no constraints of its own */
    default KinematicsConfig getKinematicsConfig() {
        return new KinematicsConfig();
    }

    /** @return omega and alpha limits used when a path has no constraints of its own */
    default PathsConfig getPathsConfig() {
        return PathsConfig.defaults();
    }
}
