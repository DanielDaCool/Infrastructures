package frc.robot.Path;

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
 */
public record PathConstraints(double vMax, double aAcc, double aDec, double aLat,
                              double omegaMax, double alphaMax) {

    /** Share of the physical limits the profile uses (section 10, PROFILE_MARGIN). */
    public static final double PROFILE_MARGIN = 0.9;

    public PathConstraints {
        check("vMax", vMax, "m/s", "withVMax(3.0)");
        check("aAcc", aAcc, "m/s^2", "withAAcc(4.0)");
        check("aDec", aDec, "m/s^2", "withADec(4.0)");
        check("aLat", aLat, "m/s^2", "withALat(4.0)");
        check("omegaMax", omegaMax, "rad/s", "withOmegaMax(6.0)");
        check("alphaMax", alphaMax, "rad/s^2", "withAlphaMax(12.0)");
    }

    private static void check(String name, double value, String unit, String change) {
        requirePositive("PathConstraints", name, value, unit, "PathConstraints.defaults()",
                "PathConstraints.defaults()." + change);
    }

    /**
     * Shared range check of the config records (section 9.12, V16): throws unless value is a finite
     * number above 0.
     *
     * @param record  record name, for example "PathConstraints"
     * @param field   field name, for example "vMax"
     * @param value   the value to check
     * @param unit    its unit, for example "m/s"
     * @param factory the factory to start from, for example "PathConstraints.defaults()"
     * @param example a complete example call, for example "PathConstraints.defaults().withVMax(3.0)"
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
     * Placeholder limits for a typical swerve robot: 4.05 m/s, 7.2 m/s^2 (acceleration, braking and
     * lateral), 2*pi rad/s and 4*pi rad/s^2. Measure your robot and change them, for example
     * {@code PathConstraints.defaults().withVMax(3.0)}. The plan speeds up with a constant aAcc; on a
     * torque-limited drivetrain the robot trails the plan above about 1.5 m/s, so lower aAcc with
     * {@link #withAAcc} if that matters.
     *
     * @return the placeholder constraints
     */
    public static PathConstraints defaults() {
        return new PathConstraints(4.05, 7.2, 7.2, 7.2, 2.0 * Math.PI, 4.0 * Math.PI);
    }

    /** @return a copy with vMax changed */
    public PathConstraints withVMax(double vMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax);
    }

    /** @return a copy with aAcc changed */
    public PathConstraints withAAcc(double aAcc) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax);
    }

    /** @return a copy with aDec changed */
    public PathConstraints withADec(double aDec) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax);
    }

    /** @return a copy with aLat changed */
    public PathConstraints withALat(double aLat) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax);
    }

    /** @return a copy with omegaMax changed */
    public PathConstraints withOmegaMax(double omegaMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax);
    }

    /** @return a copy with alphaMax changed */
    public PathConstraints withAlphaMax(double alphaMax) {
        return new PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax);
    }
}
