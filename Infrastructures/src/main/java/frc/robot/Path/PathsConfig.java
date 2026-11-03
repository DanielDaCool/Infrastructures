package frc.robot.Path;

/**
 * Robot settings the path needs that {@code KinematicsConfig} does not have (design doc section
 * 9.6). Immutable; each {@code withX} returns a new record.
 *
 * @param maxAngularVelocity robot omega limit [rad/s]
 * @param maxAngularAccel    robot alpha limit [rad/s^2]
 * @param maxJerk            tangential jerk of the speed command [m/s^3]
 */
public record PathsConfig(double maxAngularVelocity, double maxAngularAccel, double maxJerk) {

    public PathsConfig {
        check("maxAngularVelocity", maxAngularVelocity, "rad/s", "withMaxAngularVelocity(6.28)");
        check("maxAngularAccel", maxAngularAccel, "rad/s^2", "withMaxAngularAccel(12.5)");
        check("maxJerk", maxJerk, "m/s^3", "withMaxJerk(50.0)");
    }

    private static void check(String name, double value, String unit, String change) {
        PathConstraints.requirePositive("PathsConfig", name, value, unit, "PathsConfig.defaults()",
                "PathsConfig.defaults()." + change);
    }

    /** @return placeholders (2*pi, 4*pi, 50); measure on the robot (section 10, 12) */
    public static PathsConfig defaults() {
        return new PathsConfig(2.0 * Math.PI, 4.0 * Math.PI, 50.0);
    }

    /** @return a copy with maxAngularVelocity changed */
    public PathsConfig withMaxAngularVelocity(double radPerSec) {
        return new PathsConfig(radPerSec, maxAngularAccel, maxJerk);
    }

    /** @return a copy with maxAngularAccel changed */
    public PathsConfig withMaxAngularAccel(double radPerSec2) {
        return new PathsConfig(maxAngularVelocity, radPerSec2, maxJerk);
    }

    /** @return a copy with maxJerk changed */
    public PathsConfig withMaxJerk(double metersPerSec3) {
        return new PathsConfig(maxAngularVelocity, maxAngularAccel, metersPerSec3);
    }
}
