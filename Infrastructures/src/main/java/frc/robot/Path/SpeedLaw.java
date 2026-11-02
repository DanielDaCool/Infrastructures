package frc.robot.Path;

import java.util.ArrayList;
import java.util.List;

/** The jerk-aware online speed law (design doc section 7.13). One instance per command run. */
final class SpeedLaw {

    /** Margin added to the stopping distance [m] (7.13.4). */
    static final double STOP_DIST_MARGIN = 0.005;

    /** Cap comparison tolerance (rounding) [m/s]. */
    private static final double CAP_TOL = 1e-9;
    /** Tolerance of the consistency check between constructor arguments and the profile. */
    private static final double MATCH_TOL = 1e-12;

    /**
     * Speed and acceleration commanded by the law (7.13.2).
     *
     * @param v speed [m/s]
     * @param a acceleration [m/s^2]
     */
    record State(double v, double a) {
    }

    /**
     * Result of an offline run (7.13.6).
     *
     * @param time     planned time including the hand-over tail, without dwells [s]
     * @param guardHit true if the run hit the guard {@code 10 spanTime + PROFILE_TIME_GUARD} (a bug; C2)
     */
    record OfflineResult(double time, boolean guardHit) {
    }

    private final SpeedProfile profile;
    private final double jerkMax;
    private final double handOver;
    private final double vHand;

    // Constraint list of the current step, preallocated: junctions ahead plus the hand-over point.
    private final double[] consS;
    private final double[] consVt;
    private final double[] consD;
    private final boolean[] consEntry;
    private int consCount;

    /**
     * The law for a profile. jerkMax, the hand-over distance d_h and the hand-over speed
     * {@code v_h = kP d_h} all come from the profile, so they cannot disagree with it (8.3.7).
     *
     * @param profile speed profile (junctions, caps, aD, kP)
     */
    SpeedLaw(SpeedProfile profile) {
        if (profile == null) {
            throw new IllegalArgumentException("SpeedLaw: profile must not be null");
        }
        this.profile = profile;
        this.jerkMax = profile.constraints().jerkMax();
        this.handOver = profile.handOver();
        this.vHand = profile.kP() * handOver;
        int n = profile.spanCount() + 2;
        consS = new double[n];
        consVt = new double[n];
        consD = new double[n];
        consEntry = new boolean[n];
    }

    /**
     * Same as {@link #SpeedLaw(SpeedProfile)}; the arguments must equal the profile's own values
     * (its constraints and its hand-over distance), else IllegalArgumentException.
     *
     * @param profile  speed profile (junctions, caps, aD)
     * @param jerkMax  jerk limit [m/s^3]
     * @param aAcc     forward acceleration limit [m/s^2]
     * @param aDec     braking limit [m/s^2]
     * @param handOver hand-over distance d_h before s_goal [m] (8.3.7)
     */
    SpeedLaw(SpeedProfile profile, double jerkMax, double aAcc, double aDec, double handOver) {
        this(profile);
        checkMatches(profile, jerkMax, aAcc, aDec, handOver);
    }

    private static void checkMatches(SpeedProfile profile, double jerkMax, double aAcc, double aDec,
                                     double handOver) {
        PathConstraints c = profile.constraints();
        if (!(Math.abs(jerkMax - c.jerkMax()) <= MATCH_TOL) || !(Math.abs(aAcc - c.aAcc()) <= MATCH_TOL)
                || !(Math.abs(aDec - c.aDec()) <= MATCH_TOL)
                || !(Math.abs(handOver - profile.handOver()) <= MATCH_TOL)) {
            throw new IllegalArgumentException("SpeedLaw: jerkMax " + jerkMax + ", aAcc " + aAcc + ", aDec " + aDec
                    + ", handOver " + handOver + " differ from the profile's (" + c.jerkMax() + ", " + c.aAcc()
                    + ", " + c.aDec() + ", " + profile.handOver() + ")");
        }
    }

    /**
     * One loop of the law (7.13.9). A non-finite s, sGoal, cap or dt, or dt &lt;= 0, holds the state.
     *
     * @param state current (v, a)
     * @param s     lag-led progress s_v, clamped at sGoal - d_h
     * @param sGoal first stop point greater than s, or L
     * @param cap   speed cap for this loop (e.g. vMax or the profile cap at s)
     * @param dt    loop period [s]
     * @return the new (v, a)
     */
    State step(State state, double s, double sGoal, double cap, double dt) {
        if (!Double.isFinite(s) || !Double.isFinite(sGoal) || !Double.isFinite(cap) || !Double.isFinite(dt)
                || !(dt > 0.0)) {
            return state;
        }
        double v = state.v();
        double a = state.a();
        double jdt = jerkMax * dt;

        int n = profile.fillJunctionsAhead(s, sGoal, consS, consVt, consD, consEntry);
        double sHand = sGoal - handOver;
        consS[n] = sHand;
        consVt[n] = vHand;
        consD[n] = profile.brakeLimit(s, sHand);
        consEntry[n] = true;
        consCount = n + 1;

        boolean onArc = profile.onConstantArc(s);
        double aDcur = profile.decelAt(s);
        double failedD = Double.NEGATIVE_INFINITY;

        int nCand = onArc ? 2 : 3;
        for (int c = 0; c < nCand; c++) {
            boolean up = !onArc && c == 0;
            double a1;
            if (up) {
                a1 = Math.min(a + jdt, profile.accelLimit(v, s));
            } else if ((onArc && c == 0) || (!onArc && c == 1)) {
                a1 = a - Math.signum(a) * Math.min(Math.abs(a), jdt); // toward 0
            } else {
                a1 = a; // keep
            }
            a1 = Math.max(a1, -aDcur); // traction clamp
            double v1 = v + 0.5 * (a + a1) * dt;
            if (v1 < 0.0) {
                continue;
            }
            if (!up && a1 < -Math.sqrt(2.0 * jerkMax * v1)) {
                continue; // landing rule
            }
            double s1 = s + 0.5 * (v + v1) * dt;
            // safe(): cap tests, then every constraint; failed constraints raise failedD
            boolean ok = !(v1 > cap + CAP_TOL && a1 >= 0.0) && !(a1 > 0.0 && v1 + a1 * a1 / (2.0 * jerkMax) > cap + CAP_TOL);
            double ap = Math.max(0.0, a1);
            for (int i = 0; i < consCount; i++) {
                double vt = consVt[i];
                if (v1 <= vt && a1 <= 0.0) {
                    continue;
                }
                if (!consEntry[i] && v1 + ap * ap / (2.0 * jerkMax) <= vt) {
                    continue; // line-to-line junction: speed only
                }
                if (stopDist(v1, a1, vt, jerkMax, consD[i]) + STOP_DIST_MARGIN > consS[i] - s1) {
                    ok = false;
                    failedD = Math.max(failedD, consD[i]);
                }
            }
            if (ok) {
                return new State(v1, a1);
            }
        }

        double dBrake = failedD == Double.NEGATIVE_INFINITY ? consD[0] : failedD;
        dBrake = Math.min(dBrake, aDcur);
        double a1 = Math.min(a + jdt, Math.max(Math.max(a - jdt, -dBrake), -Math.sqrt(2.0 * jerkMax * Math.max(v, 0.0))));
        a1 = Math.max(a1, -aDcur); // traction clamp on the brake candidate too (traction before jerk)
        double v1 = v + 0.5 * (a + a1) * dt;
        if (v1 <= 0.0) {
            v1 = 0.0;
            a1 = Math.max(a1, 0.0); // speed floor
        }
        return new State(v1, a1);
    }

    /**
     * Distance needed to go from (v, a) to speed vt with jerk jerkMax and braking limit D (7.13.3).
     * Allocation-free; runs every loop.
     *
     * @return stopping distance [m]
     */
    static double stopDist(double v, double a, double vt, double jerkMax, double decel) {
        if (!(jerkMax > 0.0) || !(decel > 0.0)) {
            throw new IllegalArgumentException("stopDist: jerkMax and D must be > 0 (V16)");
        }
        if (v <= vt && a <= 0.0) {
            return 0.0; // already at or below vt and not speeding up
        }
        double d = 0.0;
        if (a > 0.0) { // phase 0: ramp a down to 0
            d = v * a / jerkMax + a * a * a / (3.0 * jerkMax * jerkMax);
            v += a * a / (2.0 * jerkMax);
            a = 0.0;
        }
        if (v <= vt && a == 0.0) {
            return d;
        }
        double b = -a;
        if (b > 0.0 && v - vt <= b * b / (2.0 * jerkMax)) { // over-braking: ramp out now
            double t1 = (b - PathMath.vsqrt(b * b - 2.0 * jerkMax * (v - vt))) / jerkMax;
            return d + v * t1 - b * t1 * t1 / 2.0 + jerkMax * t1 * t1 * t1 / 6.0;
        }
        double vv = v + b * b / (2.0 * jerkMax); // virtual start
        double dv = vv - vt;
        double ds;
        if (dv >= decel * decel / jerkMax) { // full D
            ds = (vv * vv - vt * vt) / (2.0 * decel) + (vv + vt) * decel / (2.0 * jerkMax);
        } else { // triangular
            ds = (vv + vt) * PathMath.vsqrt(dv / jerkMax);
        }
        double tr = b / jerkMax;
        return d + ds - (vv * tr - b * b * b / (6.0 * jerkMax * jerkMax));
    }

    /**
     * Stopping distance and time (7.13.3), for tests and logging; allocates.
     *
     * @return {distance [m], time [s]}
     */
    static double[] stopDistTime(double v, double a, double vt, double jerkMax, double decel) {
        double d = stopDist(v, a, vt, jerkMax, decel);
        if (v <= vt && a <= 0.0) {
            return new double[] {0.0, 0.0};
        }
        double t = 0.0;
        if (a > 0.0) {
            t = a / jerkMax;
            v += a * a / (2.0 * jerkMax);
            a = 0.0;
        }
        if (v <= vt && a == 0.0) {
            return new double[] {d, t};
        }
        double b = -a;
        if (b > 0.0 && v - vt <= b * b / (2.0 * jerkMax)) {
            return new double[] {d, t + (b - PathMath.vsqrt(b * b - 2.0 * jerkMax * (v - vt))) / jerkMax};
        }
        double dv = v + b * b / (2.0 * jerkMax) - vt;
        double ts = dv >= decel * decel / jerkMax ? dv / decel + decel / jerkMax : 2.0 * PathMath.vsqrt(dv / jerkMax);
        return new double[] {d, t + ts - b / jerkMax};
    }

    /**
     * Runs the law offline over the profile to get the planned time, without dwells (7.13.6). The
     * hand-over tail is included; kP and posTol are the profile's.
     *
     * @param profile  speed profile
     * @param jerkMax  jerk limit; must equal the profile's [m/s^3]
     * @param aAcc     forward acceleration limit; must equal the profile's [m/s^2]
     * @param aDec     braking limit; must equal the profile's [m/s^2]
     * @param handOver hand-over distance d_h; must equal the profile's [m]
     * @param v0       start speed [m/s]
     * @param sStart   start s
     * @param sGoal    end s (a stop or L)
     * @param dt       step [s]
     * @return time to go from sStart to within posTol of sGoal [s]
     */
    static double offlineTime(SpeedProfile profile, double jerkMax, double aAcc, double aDec,
                              double handOver, double v0, double sStart, double sGoal, double dt) {
        checkMatches(profile, jerkMax, aAcc, aDec, handOver);
        return offlineRun(profile, v0, sStart, sGoal, dt).time();
    }

    /**
     * Offline run with ideal tracking ({@code s += v dt}, tau_lag = 0) until the hand-over (at
     * {@code d_go <= d_h}, or at rest within {@code d_h + 2 STOP_DIST_MARGIN}, 8.3.7e), plus the
     * hand-over tail {@code max(0, ln(d_h/posTol)/kP)} (7.10.5, 7.13.6). Guarded at
     * {@code 10 spanTime + PROFILE_TIME_GUARD} of simulated time; then the guard time is returned and
     * guardHit is set.
     *
     * @param profile speed profile (gives jerkMax, d_h, kP, posTol)
     * @param v0      start speed [m/s]
     * @param sStart  start s
     * @param sGoal   end s (a stop or L)
     * @param dt      step [s]
     * @return the time and the guard flag
     */
    static OfflineResult offlineRun(SpeedProfile profile, double v0, double sStart, double sGoal, double dt) {
        SpeedLaw law = new SpeedLaw(profile);
        double dh = law.handOver;
        double guard = 10.0 * profile.spanTime() + SpeedProfile.PROFILE_TIME_GUARD;
        double s = sStart;
        State st = new State(Math.max(0.0, v0), 0.0);
        double t = 0.0;
        while (sGoal - s > dh && !(st.v() == 0.0 && t > 0.0 && sGoal - s <= dh + 2.0 * STOP_DIST_MARGIN)) {
            double sv = Math.min(s, sGoal - dh);
            State next = law.step(st, sv, sGoal, profile.capAt(sv), dt);
            s += 0.5 * (st.v() + next.v()) * dt;
            st = next;
            t += dt;
            if (t > guard) {
                return new OfflineResult(guard, true);
            }
        }
        if (dh > profile.posTol()) {
            t += Math.log(dh / profile.posTol()) / profile.kP(); // hand-over tail per goal
        }
        return new OfflineResult(t, false);
    }

    /** @return the junction constraints used by the last step, for tests (7.13.9 {@code cons}); built on demand */
    List<SpeedProfile.Junction> lastConstraints() {
        List<SpeedProfile.Junction> out = new ArrayList<>(consCount);
        for (int i = 0; i < consCount; i++) {
            out.add(new SpeedProfile.Junction(consS[i], consVt[i], consD[i], consEntry[i]));
        }
        return out;
    }

    /** @return the hand-over distance d_h [m] */
    double handOver() {
        return handOver;
    }
}
