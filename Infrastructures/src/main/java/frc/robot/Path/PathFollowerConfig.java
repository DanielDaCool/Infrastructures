package frc.robot.Path;

import java.util.Objects;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Follower gains, tolerances, timeout and alliance flip (design doc sections 9.6, 10).
 * Immutable; each {@code withX} returns a new record.
 *
 * @param kN            cross-track gain k_n [1/s] (8.3.5)
 * @param kTheta        heading gain k_theta [1/s] (8.5.2)
 * @param kP            goal-capture gain k_p [1/s]; keep equal to kN (8.3.7)
 * @param tauLag        lookahead time tau_lag [s] (8.4)
 * @param posTol        finish and stop-arrival position tolerance [m] (8.6, 8.7.2)
 * @param angTolRad     finish heading tolerance [rad] (8.7.2)
 * @param tSettle       finish settle time [s] (8.7.2)
 * @param startTol      largest start distance from P0 [m] (8.2.5)
 * @param timeoutFactor t_timeout = timeoutFactor * profileTime + timeoutMargin (8.7.3)
 * @param timeoutMargin [s]
 * @param flipMode      NONE / MIRROR / ROTATE (5.13, 9.9)
 * @param fieldLength   FL [m]
 * @param fieldWidth    FW [m]
 */
public record PathFollowerConfig(
        double kN,
        double kTheta,
        double kP,
        double tauLag,
        double posTol,
        double angTolRad,
        double tSettle,
        double startTol,
        double timeoutFactor,
        double timeoutMargin,
        FlipMode flipMode,
        double fieldLength,
        double fieldWidth) {

    /** Default cross-track gain k_n [1/s] (section 10). */
    static final double DEFAULT_KN = 3.0;
    /** Default heading gain k_theta [1/s] (section 10). */
    static final double DEFAULT_KTHETA = 4.0;
    /** Default goal-capture gain k_p [1/s] (section 10). */
    static final double DEFAULT_KP = 3.0;
    /** Default lookahead time tau_lag [s] (section 10). */
    static final double DEFAULT_TAU_LAG = 0.05;
    /** Default position tolerance [m] (section 10, decision 10). */
    static final double DEFAULT_POS_TOL = 0.03;
    /** Default finish heading tolerance, 2 deg [rad] (section 10, decision 10). */
    static final double DEFAULT_ANG_TOL = Math.toRadians(2.0);
    /** Default settle time [s] (section 10, decision 10). */
    static final double DEFAULT_SETTLE = 0.1;
    /** Default start tolerance [m] (section 10). */
    static final double DEFAULT_START_TOL = 0.5;
    /** Default timeout factor (section 10, decision 10). */
    static final double DEFAULT_TIMEOUT_FACTOR = 1.5;
    /** Default timeout margin [s] (section 10, decision 10). */
    static final double DEFAULT_TIMEOUT_MARGIN = 1.0;
    /** Placeholder field length [m], used only if the default AprilTag layout cannot be loaded (section 10). */
    static final double FALLBACK_FIELD_LENGTH = 16.54;
    /** Placeholder field width [m], used only if the default AprilTag layout cannot be loaded (section 10). */
    static final double FALLBACK_FIELD_WIDTH = 8.07;

    public PathFollowerConfig {
        requirePositive("kN", kN, "1/s", "withKN(3.0)");
        requirePositive("kTheta", kTheta, "1/s", "withKTheta(4.0)");
        requirePositive("kP", kP, "1/s", "withKP(3.0)");
        requirePositive("tauLag", tauLag, "s", "withTauLag(0.05)");
        requirePositive("posTol", posTol, "m", "withPosTol(0.03)");
        requirePositive("angTolRad", angTolRad, "rad", "withAngTolDegrees(2.0)");
        requirePositive("tSettle", tSettle, "s", "withSettleTime(0.1)");
        requirePositive("startTol", startTol, "m", "withStartTol(0.5)");
        requirePositive("timeoutFactor", timeoutFactor, "-", "withTimeout(1.5, 1.0)");
        requirePositive("timeoutMargin", timeoutMargin, "s", "withTimeout(1.5, 1.0)");
        Objects.requireNonNull(flipMode, Point.nullMessage("PathFollowerConfig", "the flip mode",
                "a FlipMode (for example FlipMode.ROTATE)"));
        requirePositive("fieldLength", fieldLength, "m", "withField(16.54, 8.07)");
        requirePositive("fieldWidth", fieldWidth, "m", "withField(16.54, 8.07)");
    }

    private static void requirePositive(String name, double value, String unit, String change) {
        ConfigCheck.requirePositive("PathFollowerConfig", name, value, unit, "PathFollowerConfig.defaults()",
                "PathFollowerConfig.defaults()." + change);
    }

    /**
     * Section 10 values (k_n 3, k_theta 4, k_p 3, tau_lag 0.05, pos_tol 0.03, ang_tol 2 deg,
     * t_settle 0.1, startTol 0.5, timeout 1.5 / 1.0, ROTATE); field size from the default
     * AprilTag layout (loaded once; the placeholder 16.54 x 8.07 m if it cannot be loaded).
     *
     * @return the defaults
     */
    public static PathFollowerConfig defaults() {
        return new PathFollowerConfig(DEFAULT_KN, DEFAULT_KTHETA, DEFAULT_KP, DEFAULT_TAU_LAG, DEFAULT_POS_TOL,
                DEFAULT_ANG_TOL, DEFAULT_SETTLE, DEFAULT_START_TOL, DEFAULT_TIMEOUT_FACTOR, DEFAULT_TIMEOUT_MARGIN,
                FlipMode.ROTATE, DefaultField.LENGTH, DefaultField.WIDTH);
    }

    /** Field size of {@code AprilTagFieldLayout.loadField(kDefaultField)}, read once (as RobotPose.java:94). */
    private static final class DefaultField {
        static final double LENGTH;
        static final double WIDTH;

        static {
            double length = FALLBACK_FIELD_LENGTH;
            double width = FALLBACK_FIELD_WIDTH;
            try {
                AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                if (layout.getFieldLength() > 0.0 && layout.getFieldWidth() > 0.0) {
                    length = layout.getFieldLength();
                    width = layout.getFieldWidth();
                }
            } catch (RuntimeException e) {
                // keep the placeholder pair (section 10)
            }
            LENGTH = length;
            WIDTH = width;
        }

        private DefaultField() {
        }
    }

    /** @return a copy with kN changed */
    public PathFollowerConfig withKN(double kN) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with kTheta changed */
    public PathFollowerConfig withKTheta(double kTheta) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with kP changed */
    public PathFollowerConfig withKP(double kP) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with tauLag changed */
    public PathFollowerConfig withTauLag(double seconds) {
        return new PathFollowerConfig(kN, kTheta, kP, seconds, posTol, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with posTol changed */
    public PathFollowerConfig withPosTol(double meters) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, meters, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with angTol changed (given in degrees) */
    public PathFollowerConfig withAngTolDegrees(double degrees) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, Math.toRadians(degrees), tSettle,
                startTol, timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with tSettle changed */
    public PathFollowerConfig withSettleTime(double seconds) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, seconds, startTol,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with startTol changed */
    public PathFollowerConfig withStartTol(double meters) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, meters,
                timeoutFactor, timeoutMargin, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with the timeout factor and margin changed */
    public PathFollowerConfig withTimeout(double factor, double marginSeconds) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, startTol,
                factor, marginSeconds, flipMode, fieldLength, fieldWidth);
    }

    /** @return a copy with the flip mode changed */
    public PathFollowerConfig withFlip(FlipMode mode) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, mode, fieldLength, fieldWidth);
    }

    /** @return a copy with the field size changed */
    public PathFollowerConfig withField(double lengthMeters, double widthMeters) {
        return new PathFollowerConfig(kN, kTheta, kP, tauLag, posTol, angTolRad, tSettle, startTol,
                timeoutFactor, timeoutMargin, flipMode, lengthMeters, widthMeters);
    }
}
