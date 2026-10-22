package frc.robot.Kinematics;

/**
 * Settings for {@link DemaciaKinematics#toModuleCommands}.
 *
 * @param latencySeconds                       command + gyro latency + dt / 2, measured on the robot [s]:
 *                                             the heading is advanced by gyro rate times this
 * @param measuredAngleLatencySeconds          how old the measured module angle is [s]: the angle is
 *                                             advanced by the measured steer velocity times this
 * @param maxSteerRateRadiansPerSecond         the fastest a module can steer; clamps the steer velocity
 *                                             feed-forward [rad/s]
 * @param maxDriveAccelMetersPerSecondSquared  the largest drive acceleration feed-forward sent [m/s^2]
 * @param steerVelocityFeedforward             true only for a PositionVoltage steer; with MotionMagic
 *                                             leave it false
 */
public record ModuleCommandConfig(double latencySeconds, double measuredAngleLatencySeconds,
        double maxSteerRateRadiansPerSecond, double maxDriveAccelMetersPerSecondSquared,
        boolean steerVelocityFeedforward) {

    public ModuleCommandConfig {
        if (!(latencySeconds >= 0 && Double.isFinite(latencySeconds))) {
            throw new IllegalArgumentException("latencySeconds must be finite and >= 0: " + latencySeconds);
        }
        if (!(measuredAngleLatencySeconds >= 0 && Double.isFinite(measuredAngleLatencySeconds))) {
            throw new IllegalArgumentException(
                    "measuredAngleLatencySeconds must be finite and >= 0: " + measuredAngleLatencySeconds);
        }
        if (!(maxSteerRateRadiansPerSecond > 0 && Double.isFinite(maxSteerRateRadiansPerSecond))) {
            throw new IllegalArgumentException(
                    "maxSteerRateRadiansPerSecond must be finite and > 0: " + maxSteerRateRadiansPerSecond);
        }
        if (!(maxDriveAccelMetersPerSecondSquared > 0 && Double.isFinite(maxDriveAccelMetersPerSecondSquared))) {
            throw new IllegalArgumentException("maxDriveAccelMetersPerSecondSquared must be finite and > 0: "
                    + maxDriveAccelMetersPerSecondSquared);
        }
    }

    /** 35 ms for both latencies, 12 rad/s steer, 20 m/s^2 drive, no steer velocity feed-forward. */
    public static ModuleCommandConfig defaults() {
        return new ModuleCommandConfig(0.035, 0.035, 12, 20, false);
    }
}
