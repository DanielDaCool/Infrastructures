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
        requirePositive("maxAngularVelocity", maxAngularVelocity);
        requirePositive("maxAngularAccel", maxAngularAccel);
        requirePositive("maxJerk", maxJerk);
        if (!Double.isFinite(moduleRadius) || moduleRadius < 0.0) {
            throw new IllegalArgumentException(
                    "PathsConfig.moduleRadius must be finite and >= 0, got " + moduleRadius + " (V16)");
        }
    }

    private static void requirePositive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "PathsConfig." + name + " must be finite and > 0, got " + value + " (V16)");
        }
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
        double r = 0.0;
        for (Translation2d m : modules) {
            r = Math.max(r, m.getNorm());
        }
        return defaults().withModuleRadius(r);
    }
}
