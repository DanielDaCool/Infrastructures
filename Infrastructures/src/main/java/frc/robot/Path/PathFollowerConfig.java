package frc.robot.Path;

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

    public PathFollowerConfig {
        // TODO phase 3: every number finite and > 0 (V16), flipMode not null (V3)
    }

    /**
     * Section 10 values (k_n 3, k_theta 4, k_p 3, tau_lag 0.05, pos_tol 0.03, ang_tol 2 deg,
     * t_settle 0.1, startTol 0.5, timeout 1.5 / 1.0, ROTATE); field size from the default
     * AprilTag layout.
     *
     * @return the defaults
     */
    public static PathFollowerConfig defaults() {
        throw new UnsupportedOperationException("TODO phase 3");
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
