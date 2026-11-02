package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Robot settings the path needs that {@code KinematicsConfig} does not have (design doc section
 * 9.6). Immutable; each {@code withX} returns a new record.
 *
 * @param maxAngularVelocity robot omega limit [rad/s]
 * @param maxAngularAccel    robot alpha limit [rad/s^2]
 * @param maxJerk            tangential jerk of the speed command [m/s^3]
 * @param moduleRadius       robot center to the farthest module [m]; 0 = unknown, wheel-speed cap skipped
 */
public record PathsConfig(
        double maxAngularVelocity,
        double maxAngularAccel,
        double maxJerk,
        double moduleRadius) {

    public PathsConfig {
        requirePositive("maxAngularVelocity", maxAngularVelocity, "rad/s", "withMaxAngularVelocity(6.28)");
        requirePositive("maxAngularAccel", maxAngularAccel, "rad/s^2", "withMaxAngularAccel(12.5)");
        requirePositive("maxJerk", maxJerk, "m/s^3", "withMaxJerk(50.0)");
        ConfigCheck.requireNotNegative("PathsConfig", "moduleRadius", moduleRadius, "meters", "PathsConfig.defaults()",
                "PathsConfig.defaults().withModuleRadius(0.45)");
    }

    private static void requirePositive(String name, double value, String unit, String change) {
        ConfigCheck.requirePositive("PathsConfig", name, value, unit, "PathsConfig.defaults()",
                "PathsConfig.defaults()." + change);
    }

    /** @return placeholders (2*pi, 4*pi, 50, 0); measure on the robot (section 10, 12) */
    public static PathsConfig defaults() {
        return new PathsConfig(2.0 * Math.PI, 4.0 * Math.PI, 50.0, 0.0);
    }

    /** @return a copy with maxAngularVelocity changed */
    public PathsConfig withMaxAngularVelocity(double radPerSec) {
        return new PathsConfig(radPerSec, maxAngularAccel, maxJerk, moduleRadius);
    }

    /** @return a copy with maxAngularAccel changed */
    public PathsConfig withMaxAngularAccel(double radPerSec2) {
        return new PathsConfig(maxAngularVelocity, radPerSec2, maxJerk, moduleRadius);
    }

    /** @return a copy with maxJerk changed */
    public PathsConfig withMaxJerk(double metersPerSec3) {
        return new PathsConfig(maxAngularVelocity, maxAngularAccel, metersPerSec3, moduleRadius);
    }

    /** @return a copy with moduleRadius changed */
    public PathsConfig withModuleRadius(double meters) {
        return new PathsConfig(maxAngularVelocity, maxAngularAccel, maxJerk, meters);
    }

    /**
     * {@link #defaults()} with moduleRadius = the largest module distance from the robot center.
     *
     * @param modules module positions relative to the robot center
     * @return the config
     */
    public static PathsConfig fromModules(Translation2d... modules) {
        if (modules == null) {
            throw new NullPointerException("PathsConfig.fromModules: the module list is null; pass the module positions"
                    + " (for example PathsConfig.fromModules(new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),"
                    + " new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3)))");
        }
        double r = 0.0;
        for (Translation2d m : modules) {
            if (m == null) {
                throw new NullPointerException("PathsConfig.fromModules: a module position is null; pass a"
                        + " Translation2d for every module (for example new Translation2d(0.3, 0.3))");
            }
            r = Math.max(r, m.getNorm());
        }
        return defaults().withModuleRadius(r);
    }
}
