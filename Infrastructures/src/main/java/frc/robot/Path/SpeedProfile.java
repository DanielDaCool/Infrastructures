package frc.robot.Path;

import java.util.List;

import frc.robot.Kinematics.KinematicsConfig;

/**
 * Spans, caps, passes, constant-speed and ARC_VAR arcs, sub-profiles, vBack / vProf and time
 * (design doc section 7). Built at command start.
 */
final class SpeedProfile {

    /** Grip share of an ARC_VAR arc (7.8). */
    static final double ARC_RHO = 0.8;
    /** Heading lag allowed, 5 deg [rad] (6.6, 7.4.6). */
    static final double HEADING_LAG_ALLOWED = Math.toRadians(5.0);
    /** Face-target sub-span length [m] (6.9, 7.3.1). */
    static final double FT_SPAN = 0.10;
    /** Span merge tolerance [m] (7.3.1). */
    static final double S_EPS = 1e-6;
    /** Share of alphaMax per face-target term (6.9). */
    static final double FACE_TARGET_ALPHA_SPLIT = 0.5;
    /** Motor-curve inversion bisection iterations (7.9). */
    static final int MOTOR_CURVE_ITER = 50;
    /** Room-to-brake bisection iterations (7.8). */
    static final int ROOM_TO_BRAKE_ITER = 60;
    /** Offline profileTime guard is {@code 10 * spanTime + this} [s] (7.13.6). */
    static final double PROFILE_TIME_GUARD = 10.0;

    /**
     * A junction ahead: a place where the speed must be at most vBack, with the braking
     * acceleration usable before it (7.6, 7.13.4).
     *
     * @param s        position [m]
     * @param vBack    backward-pass speed at the junction [m/s]
     * @param decel    D_j: braking limit of the span before it [m/s^2]
     * @param arcEntry true if the junction needs a = 0 (arc entry or hand-over point)
     */
    record Junction(double s, double vBack, double decel, boolean arcEntry) {
    }

    /**
     * Builds the profile (7.17).
     *
     * @param table        geometry (active frame)
     * @param heading      heading profile (omega and alpha caps)
     * @param constraints  resolved constraints
     * @param kinematics   motor curve (7.9); may be null to skip it
     * @param moduleRadius R_mod for the wheel-speed cap, 0 = skip (7.11)
     * @param v0           measured start speed along T(0), at least 0 (7.6.4)
     * @return the profile
     */
    static SpeedProfile build(SegmentTable table, HeadingProfile heading, PathConstraints constraints,
                              KinematicsConfig kinematics, double moduleRadius, double v0) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return number of spans (7.3) */
    int spanCount() {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return s at the start of span k */
    double spanStart(int k) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return length of span k */
    double spanLength(int k) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return the span containing s (7.13.10 step 1) */
    int spanAt(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return backward-pass speed limit at s, {@code min(vCap, sqrt(vB[k+1]^2 + 2 aD (l - q)))} (7.13.10) */
    double vBack(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return full profile speed at s, for logging (7.13.10 step 4) */
    double vProf(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return speed cap vCap of the span at s (7.4) */
    double capAt(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return braking limit aD of the span at s (7.13.9: its own aD on a line or ARC_VAR, aDec on a constant arc) */
    double decelAt(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return Amax(v, s): acceleration limit at speed v in the span at s, with the motor curve (7.9, 7.13.9) */
    double accelLimit(double v, double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return true if s is on a constant-speed arc, where the law has no "up" candidate (7.13.9) */
    boolean onConstantArc(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return junctions with s in (sFrom, sGoal), ascending, for the law's constraint list (7.13.9) */
    List<Junction> junctionsAhead(double sFrom, double sGoal) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return the first stop s greater than s, or L if none: s_goal, selects the sub-profile (7.10, 8.14) */
    double nextStopS(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return dwell [s] of the stop at sStop; the dwell at L is the last point's (7.10.4) */
    double dwellAt(double sStop) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return the planned time including dwells, from the offline jerk law (7.12, 7.13.6, 7.10.5) */
    double profileTime() {
        throw new UnsupportedOperationException("TODO phase 2");
    }
}
