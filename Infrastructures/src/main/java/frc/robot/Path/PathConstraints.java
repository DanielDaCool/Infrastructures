package frc.robot.Path;

import frc.robot.Kinematics.KinematicsConfig;

/**
 * Motion limits of a path (design doc sections 7.2, 9.5). Immutable; each {@code withX} returns a
 * new record.
 *
 * @param vMax     top speed [m/s]
 * @param aAcc     forward acceleration limit [m/s^2]
 * @param aDec     braking limit [m/s^2]
 * @param aLat     lateral acceleration limit [m/s^2]
 * @param omegaMax robot angular velocity limit [rad/s]
 * @param alphaMax robot angular acceleration limit [rad/s^2]
 * @param jerkMax  tangential jerk limit [m/s^3]
 */
public record PathConstraints(double vMax, double aAcc, double aDec, double aLat,
                              double omegaMax, double alphaMax, double jerkMax) {

    /** Share of the physical limits the profile uses (section 10, PROFILE_MARGIN). */
    public static final double PROFILE_MARGIN = 0.9;

    public PathConstraints {
        requirePositive("vMax", vMax);
        requirePositive("aAcc", aAcc);
        requirePositive("aDec", aDec);
        requirePositive("aLat", aLat);
        requirePositive("omegaMax", omegaMax);
        requirePositive("alphaMax", alphaMax);
        requirePositive("jerkMax", jerkMax);
    }

    private static void requirePositive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "PathConstraints." + name + " must be finite and > 0, got " + value + " (V16)");
        }
    }

    /**
     * Derives the limits from the Kinematics config (section 9.5).
     *
     * @param config robot limits
     * @param paths  the settings Kinematics does not have
     * @return the constraints
     */
    public static PathConstraints fromConfig(KinematicsConfig config, PathsConfig paths) {
        if (config == null || paths == null) {
            throw new IllegalArgumentException("PathConstraints.fromConfig: config and paths must not be null");
        }
        double tilt = Math.min(config.maxTiltAccelFront, config.maxTiltAccelSide);
        double brake = Math.min(config.maxSkidAccel, tilt);
        double vMax = PROFILE_MARGIN * Math.min(config.maxAllowedModuleVelocity, config.freeSpeed);
        double aAcc = PROFILE_MARGIN * Math.min(config.maxForwardAccel, brake);
        double aDec = PROFILE_MARGIN * brake;
        return new PathConstraints(vMax, aAcc, aDec, aDec,
                paths.maxAngularVelocity(), paths.maxAngularAccel(), paths.maxJerk());
    }

    /**
     * Same as {@code fromConfig(config, PathsConfig.defaults())}.
     *
     * @param config robot limits
     * @return the constraints
     */
    public static PathConstraints fromConfig(KinematicsConfig config) {
        return fromConfig(config, PathsConfig.defaults());
    }

    /** @return a copy with vMax changed */
    public PathConstraints withVMax(double vMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }

    /** @return a copy with aAcc changed */
    public PathConstraints withAAcc(double aAcc) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }

    /** @return a copy with aDec changed */
    public PathConstraints withADec(double aDec) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }

    /** @return a copy with aLat changed */
    public PathConstraints withALat(double aLat) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }

    /** @return a copy with omegaMax changed */
    public PathConstraints withOmegaMax(double omegaMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }

    /** @return a copy with alphaMax changed */
    public PathConstraints withAlphaMax(double alphaMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }

    /** @return a copy with jerkMax changed */
    public PathConstraints withJerkMax(double jerkMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax, jerkMax);
    }
}
