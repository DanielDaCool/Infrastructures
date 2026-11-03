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
        check("vMax", vMax, "m/s", "withVMax(3.0)");
        check("aAcc", aAcc, "m/s^2", "withAAcc(4.0)");
        check("aDec", aDec, "m/s^2", "withADec(4.0)");
        check("aLat", aLat, "m/s^2", "withALat(4.0)");
        check("omegaMax", omegaMax, "rad/s", "withOmegaMax(6.0)");
        check("alphaMax", alphaMax, "rad/s^2", "withAlphaMax(12.0)");
        check("jerkMax", jerkMax, "m/s^3", "withJerkMax(50.0)");
    }

    private static void check(String name, double value, String unit, String change) {
        requirePositive("PathConstraints", name, value, unit, "PathConstraints.fromConfig(config)",
                "PathConstraints.fromConfig(config)." + change);
    }

    /**
     * Shared range check of the config records (section 9.12, V16): throws unless value is a finite
     * number above 0.
     *
     * @param record  record name, for example "PathConstraints"
     * @param field   field name, for example "vMax"
     * @param value   the value to check
     * @param unit    its unit, for example "m/s"
     * @param factory the factory to start from, for example "PathConstraints.fromConfig(config)"
     * @param example a complete example call, for example "PathConstraints.fromConfig(config).withVMax(3.0)"
     */
    static void requirePositive(String record, String field, double value, String unit, String factory,
                                String example) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(record + "." + field + " must be a finite number above 0 (" + unit
                    + "), but it is " + value + ". Start from " + factory + " and change one value, for example "
                    + example + ".");
        }
    }

    /**
     * Derives the limits from the Kinematics config at {@link #PROFILE_MARGIN} (section 9.5).
     *
     * @param config robot limits
     * @param paths  the settings Kinematics does not have
     * @return the constraints
     */
    public static PathConstraints fromConfig(KinematicsConfig config, PathsConfig paths) {
        if (config == null) {
            throw new NullPointerException("PathConstraints.fromConfig: the KinematicsConfig is null; pass your robot's"
                    + " config (for example PathConstraints.fromConfig(new KinematicsConfig()))");
        }
        if (paths == null) {
            throw new NullPointerException("PathConstraints.fromConfig: the PathsConfig is null; pass one (for example"
                    + " PathConstraints.fromConfig(config, PathsConfig.defaults())), or leave it out");
        }
        double tilt = Math.min(config.maxTiltAccelFront, config.maxTiltAccelSide);
        double brake = Math.min(config.maxSkidAccel, tilt);
        double vMax = PROFILE_MARGIN * config.maxAllowedModuleVelocity;
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
