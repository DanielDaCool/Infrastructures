package frc.robot.Path;

import java.util.Objects;

/**
 * Follower gains, tolerances, timeout and alliance flip (design doc sections 9.6, 10).
 * Immutable; each {@code withX} returns a new record.
 *
 * @param kN            cross-track gain k_n [1/s] (8.3.5)
 * @param kP            goal-capture gain k_p [1/s]; keep equal to kN (8.3.7)
 * @param posTol        finish and stop-arrival position tolerance [m] (8.6, 8.7.2)
 * @param angTolRad     finish heading tolerance [rad] (8.7.2)
 * @param startTol      largest start distance from P0 [m] (8.2.5)
 * @param timeoutFactor t_timeout = timeoutFactor * profileTime + timeoutMargin (8.7.3)
 * @param timeoutMargin [s]
 * @param flipMode      NONE / MIRROR / ROTATE (5.13, 9.9)
 * @param fieldLength   FL [m], default {@link #FIELD_LENGTH}
 * @param fieldWidth    FW [m], default {@link #FIELD_WIDTH}
 */
public record PathFollowerConfig(
        double kN,
        double kP,
        double posTol,
        double angTolRad,
        double startTol,
        double timeoutFactor,
        double timeoutMargin,
        FlipMode flipMode,
        double fieldLength,
        double fieldWidth) {

    /** Heading gain k_theta [1/s] (8.5.2, section 10). */
    static final double K_THETA = 4.0;
    /** Lookahead time tau_lag [s] (8.4, section 10). */
    static final double TAU_LAG = 0.05;
    /** Finish settle time [s] (8.7.2, section 10). */
    static final double T_SETTLE = 0.1;
    /** Default goal-capture gain k_p [1/s] (section 10). */
    static final double DEFAULT_KP = 3.0;
    /** Default position tolerance [m] (section 10, decision 10). */
    static final double DEFAULT_POS_TOL = 0.03;
    /** Default field length [m]; change it with {@link #withField} for another field. */
    public static final double FIELD_LENGTH = 16.54;
    /** Default field width [m]; change it with {@link #withField} for another field. */
    public static final double FIELD_WIDTH = 8.07;

    /** How a blue path is turned into a red one (sections 5.13, 9.9). */
    public enum FlipMode {
        /** No flip; the team writes separate red paths. */
        NONE,
        /** Mirror about x = fieldLength / 2. */
        MIRROR,
        /** Rotate 180 degrees about the field center. */
        ROTATE
    }

    public PathFollowerConfig {
        check("kN", kN, "1/s", "withKN(3.0)");
        check("kP", kP, "1/s", "withKP(3.0)");
        check("posTol", posTol, "m", "withPosTol(0.03)");
        check("angTolRad", angTolRad, "rad", "withAngTolDegrees(2.0)");
        check("startTol", startTol, "m", "withStartTol(0.5)");
        check("timeoutFactor", timeoutFactor, "-", "withTimeout(1.5, 1.0)");
        check("timeoutMargin", timeoutMargin, "s", "withTimeout(1.5, 1.0)");
        Objects.requireNonNull(flipMode, Point.nullMessage("PathFollowerConfig", "the flip mode",
                "a FlipMode (for example FlipMode.ROTATE)"));
        check("fieldLength", fieldLength, "m", "withField(16.54, 8.07)");
        check("fieldWidth", fieldWidth, "m", "withField(16.54, 8.07)");
    }

    private static void check(String name, double value, String unit, String change) {
        PathConstraints.requirePositive("PathFollowerConfig", name, value, unit, "PathFollowerConfig.defaults()",
                "PathFollowerConfig.defaults()." + change);
    }

    /**
     * Section 10 values (k_n 3, k_p 3, pos_tol 0.03, ang_tol 2 deg, startTol 0.5, timeout 1.5 / 1.0,
     * ROTATE, field 16.54 x 8.07 m).
     *
     * @return the defaults
     */
    public static PathFollowerConfig defaults() {
        return new PathFollowerConfig(3.0, DEFAULT_KP, DEFAULT_POS_TOL, Math.toRadians(2.0), 0.5, 1.5, 1.0,
                FlipMode.ROTATE, FIELD_LENGTH, FIELD_WIDTH);
    }

    /** @return a copy with value i (in record order, flipMode skipped) replaced by x */
    private PathFollowerConfig with(int i, double x) {
        double[] v = {kN, kP, posTol, angTolRad, startTol, timeoutFactor, timeoutMargin, fieldLength, fieldWidth};
        v[i] = x;
        return new PathFollowerConfig(v[0], v[1], v[2], v[3], v[4], v[5], v[6], flipMode, v[7], v[8]);
    }

    /** @return a copy with kN changed */
    public PathFollowerConfig withKN(double kN) {
        return with(0, kN);
    }

    /** @return a copy with kP changed */
    public PathFollowerConfig withKP(double kP) {
        return with(1, kP);
    }

    /** @return a copy with posTol changed */
    public PathFollowerConfig withPosTol(double meters) {
        return with(2, meters);
    }

    /** @return a copy with angTol changed (given in degrees) */
    public PathFollowerConfig withAngTolDegrees(double degrees) {
        return with(3, Math.toRadians(degrees));
    }

    /** @return a copy with startTol changed */
    public PathFollowerConfig withStartTol(double meters) {
        return with(4, meters);
    }

    /** @return a copy with the timeout factor and margin changed */
    public PathFollowerConfig withTimeout(double factor, double marginSeconds) {
        return with(5, factor).with(6, marginSeconds);
    }

    /** @return a copy with the flip mode changed */
    public PathFollowerConfig withFlip(FlipMode mode) {
        return new PathFollowerConfig(kN, kP, posTol, angTolRad, startTol, timeoutFactor, timeoutMargin, mode,
                fieldLength, fieldWidth);
    }

    /** @return a copy with the field size changed */
    public PathFollowerConfig withField(double lengthMeters, double widthMeters) {
        return with(7, lengthMeters).with(8, widthMeters);
    }
}
