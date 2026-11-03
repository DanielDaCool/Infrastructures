package frc.robot.Path;

/**
 * Robot settings the path needs that {@code KinematicsConfig} does not have (design doc section
 * 9.6). Immutable; each {@code withX} returns a new record.
 *
 * @param maxAngularVelocity robot omega limit [rad/s]
 * @param maxAngularAccel    robot alpha limit [rad/s^2]
 */
public record PathsConfig(double maxAngularVelocity, double maxAngularAccel) {

    public PathsConfig {
        check("maxAngularVelocity", maxAngularVelocity, "rad/s", "withMaxAngularVelocity(6.28)");
        check("maxAngularAccel", maxAngularAccel, "rad/s^2", "withMaxAngularAccel(12.5)");
    }

    private static void check(String name, double value, String unit, String change) {
        PathConstraints.requirePositive("PathsConfig", name, value, unit, "PathsConfig.defaults()",
                "PathsConfig.defaults()." + change);
    }

    /** @return placeholders (2*pi, 4*pi); measure on the robot (section 10, 12) */
    public static PathsConfig defaults() {
        return new PathsConfig(2.0 * Math.PI, 4.0 * Math.PI);
    }

    /** @return a copy with maxAngularVelocity changed */
    public PathsConfig withMaxAngularVelocity(double radPerSec) {
        return new PathsConfig(radPerSec, maxAngularAccel);
    }

    /** @return a copy with maxAngularAccel changed */
    public PathsConfig withMaxAngularAccel(double radPerSec2) {
        return new PathsConfig(maxAngularVelocity, radPerSec2);
    }
}
