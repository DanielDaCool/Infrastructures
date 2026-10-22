package frc.robot.Kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * What one module is told to do this loop.
 *
 * @param angle                              steer angle to command (after the reverse decision)
 * @param speedMetersPerSecond               drive speed to command, negative when the module is
 *                                           reversed, already scaled by the cosine of the steer error
 * @param accelerationMetersPerSecondSquared drive acceleration feed-forward, from the plan only
 *                                           (e.g. {@code VelocityVoltage.withAcceleration})
 * @param steerVelocityRadiansPerSecond      steer velocity feed-forward, 0 when it is off
 */
public record ModuleCommand(Rotation2d angle, double speedMetersPerSecond,
        double accelerationMetersPerSecondSquared, double steerVelocityRadiansPerSecond) {

    /** The angle and speed as a {@link SwerveModuleState}. */
    public SwerveModuleState toState() {
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }
}
