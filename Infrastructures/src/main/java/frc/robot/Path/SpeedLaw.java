package frc.robot.Path;

import java.util.List;

/** The jerk-aware online speed law (design doc section 7.13). One instance per command run. */
final class SpeedLaw {

    /** Margin added to the stopping distance [m] (7.13.4). */
    static final double STOP_DIST_MARGIN = 0.005;

    /**
     * Speed and acceleration commanded by the law (7.13.2).
     *
     * @param v speed [m/s]
     * @param a acceleration [m/s^2]
     */
    record State(double v, double a) {
    }

    /**
     * @param profile  speed profile (junctions, caps, aD)
     * @param jerkMax  jerk limit [m/s^3]
     * @param aAcc     forward acceleration limit [m/s^2]
     * @param aDec     braking limit [m/s^2]
     * @param handOver hand-over distance d_h before s_goal [m] (8.3.7)
     */
    SpeedLaw(SpeedProfile profile, double jerkMax, double aAcc, double aDec, double handOver) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /**
     * One loop of the law (7.13.9).
     *
     * @param state current (v, a)
     * @param s     lag-led progress s_v, clamped at sGoal - d_h
     * @param sGoal first stop point greater than s, or L
     * @param cap   speed cap for this loop (e.g. vMax or the profile cap at s)
     * @param dt    loop period [s]
     * @return the new (v, a)
     */
    State step(State state, double s, double sGoal, double cap, double dt) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /**
     * Distance needed to go from (v, a) to speed vt with jerk jerkMax and braking limit D (7.13.3).
     *
     * @return stopping distance [m]
     */
    static double stopDist(double v, double a, double vt, double jerkMax, double decel) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /**
     * Runs the law offline over the profile to get the planned time, without dwells (7.13.6).
     *
     * @param profile   speed profile
     * @param v0        start speed [m/s]
     * @param sStart    start s
     * @param sGoal     end s (a stop or L)
     * @param dt        step [s]
     * @return time to go from sStart to sGoal [s]
     */
    static double offlineTime(SpeedProfile profile, double jerkMax, double aAcc, double aDec,
                              double handOver, double v0, double sStart, double sGoal, double dt) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return the junction constraints used by the last step, for tests (7.13.9 {@code cons}) */
    List<SpeedProfile.Junction> lastConstraints() {
        throw new UnsupportedOperationException("TODO phase 2");
    }
}
