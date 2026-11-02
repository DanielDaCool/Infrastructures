package frc.robot.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** The jerk-aware online speed law (design doc section 7.13). One instance per command run. */
final class SpeedLaw {

    /** Margin added to the stopping distance [m] (7.13.4). */
    static final double STOP_DIST_MARGIN = 0.005;

    /** Cap comparison tolerance (rounding) [m/s]. */
    private static final double CAP_TOL = 1e-9;

    /**
     * Speed and acceleration commanded by the law (7.13.2).
     *
     * @param v speed [m/s]
     * @param a acceleration [m/s^2]
     */
    record State(double v, double a) {
    }

    private final SpeedProfile profile;
    private final double jerkMax;
    private final double aAcc;
    private final double aDec;
    private final double handOver;
    private final double kP;
    private List<SpeedProfile.Junction> lastCons = Collections.emptyList();

    /**
     * @param profile  speed profile (junctions, caps, aD)
     * @param jerkMax  jerk limit [m/s^3]
     * @param aAcc     forward acceleration limit [m/s^2]
     * @param aDec     braking limit [m/s^2]
     * @param handOver hand-over distance d_h before s_goal [m] (8.3.7)
     */
    SpeedLaw(SpeedProfile profile, double jerkMax, double aAcc, double aDec, double handOver) {
        this(profile, jerkMax, aAcc, aDec, handOver, SpeedProfile.DEFAULT_KP);
    }

    /**
     * Same as the five-argument constructor with the goal-capture gain given: the hand-over speed is
     * {@code v_h = kP * d_h} (8.3.7).
     *
     * @param kP goal-capture gain k_p [1/s]
     */
    SpeedLaw(SpeedProfile profile, double jerkMax, double aAcc, double aDec, double handOver, double kP) {
        if (profile == null) {
            throw new IllegalArgumentException("SpeedLaw: profile must not be null");
        }
        if (!(jerkMax > 0.0) || !Double.isFinite(jerkMax)) {
            throw new IllegalArgumentException("SpeedLaw: jerkMax must be finite and > 0 (V16)");
        }
        if (!(aDec > 0.0) || !(aAcc > 0.0) || !(handOver >= 0.0) || !(kP > 0.0)) {
            throw new IllegalArgumentException("SpeedLaw: aAcc, aDec, kP must be > 0 and handOver >= 0");
        }
        this.profile = profile;
        this.jerkMax = jerkMax;
        this.aAcc = aAcc;
        this.aDec = aDec;
        this.handOver = handOver;
        this.kP = kP;
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
        double v = state.v();
        double a = state.a();
        double jdt = jerkMax * dt;

        List<SpeedProfile.Junction> cons = new ArrayList<>(profile.junctionsAhead(s, sGoal));
        double sHand = sGoal - handOver;
        cons.add(new SpeedProfile.Junction(sHand, kP * handOver, profile.brakeLimit(s, sHand), true));
        lastCons = Collections.unmodifiableList(cons);

        boolean onArc = profile.onConstantArc(s);
        double aDcur = profile.decelAt(s);
        boolean[] failed = new boolean[cons.size()];

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
            if (safe(v1, a1, s1, cons, cap, failed)) {
                return new State(v1, a1);
            }
        }

        double dBrake = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < failed.length; i++) {
            if (failed[i]) {
                dBrake = Math.max(dBrake, cons.get(i).decel());
            }
        }
        if (dBrake == Double.NEGATIVE_INFINITY) {
            dBrake = cons.get(0).decel();
        }
        dBrake = Math.min(dBrake, aDcur);
        double a1 = Math.min(a + jdt, Math.max(Math.max(a - jdt, -dBrake), -Math.sqrt(2.0 * jerkMax * Math.max(v, 0.0))));
        double v1 = v + 0.5 * (a + a1) * dt;
        if (v1 <= 0.0) {
            v1 = 0.0;
            a1 = Math.max(a1, 0.0); // speed floor
        }
        return new State(v1, a1);
    }

    private boolean safe(double v1, double a1, double s1, List<SpeedProfile.Junction> cons, double cap,
                         boolean[] failed) {
        boolean ok = true;
        if (v1 > cap + CAP_TOL && a1 >= 0.0) {
            ok = false;
        }
        if (a1 > 0.0 && v1 + a1 * a1 / (2.0 * jerkMax) > cap + CAP_TOL) {
            ok = false;
        }
        for (int i = 0; i < cons.size(); i++) {
            SpeedProfile.Junction c = cons.get(i);
            double vt = c.vBack();
            if (v1 <= vt && a1 <= 0.0) {
                continue;
            }
            double ap = Math.max(0.0, a1);
            if (!c.arcEntry() && v1 + ap * ap / (2.0 * jerkMax) <= vt) {
                continue; // line-to-line junction: speed only
            }
            if (stopDist(v1, a1, vt, jerkMax, c.decel()) + STOP_DIST_MARGIN > c.s() - s1) {
                ok = false;
                failed[i] = true;
            }
        }
        return ok;
    }

    /**
     * Distance needed to go from (v, a) to speed vt with jerk jerkMax and braking limit D (7.13.3).
     *
     * @return stopping distance [m]
     */
    static double stopDist(double v, double a, double vt, double jerkMax, double decel) {
        return stopDistTime(v, a, vt, jerkMax, decel)[0];
    }

    /**
     * Stopping distance and time (7.13.3).
     *
     * @return {distance [m], time [s]}
     */
    static double[] stopDistTime(double v, double a, double vt, double jerkMax, double decel) {
        if (!(jerkMax > 0.0) || !(decel > 0.0)) {
            throw new IllegalArgumentException("stopDist: jerkMax and D must be > 0 (V16)");
        }
        if (v <= vt && a <= 0.0) {
            return new double[] {0.0, 0.0}; // already at or below vt and not speeding up
        }
        double d = 0.0;
        double t = 0.0;
        if (a > 0.0) { // phase 0: ramp a down to 0
            double t0 = a / jerkMax;
            d += v * t0 + a * a * a / (3.0 * jerkMax * jerkMax);
            v += a * a / (2.0 * jerkMax);
            t += t0;
            a = 0.0;
        }
        if (v <= vt && a == 0.0) {
            return new double[] {d, t};
        }
        double b = -a;
        if (b > 0.0 && v - vt <= b * b / (2.0 * jerkMax)) { // over-braking: ramp out now
            double t1 = (b - PathMath.vsqrt(b * b - 2.0 * jerkMax * (v - vt))) / jerkMax;
            return new double[] {d + v * t1 - b * t1 * t1 / 2.0 + jerkMax * t1 * t1 * t1 / 6.0, t + t1};
        }
        double vv = v + b * b / (2.0 * jerkMax); // virtual start
        double dv = vv - vt;
        double ds;
        double ts;
        if (dv >= decel * decel / jerkMax) { // full D
            ds = (vv * vv - vt * vt) / (2.0 * decel) + (vv + vt) * decel / (2.0 * jerkMax);
            ts = dv / decel + decel / jerkMax;
        } else { // triangular
            double r = PathMath.vsqrt(dv / jerkMax);
            ds = (vv + vt) * r;
            ts = 2.0 * r;
        }
        double tr = b / jerkMax;
        double dr = vv * tr - b * b * b / (6.0 * jerkMax * jerkMax);
        return new double[] {d + ds - dr, t + ts - tr};
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
        return offlineTime(profile, jerkMax, aAcc, aDec, handOver, v0, sStart, sGoal, dt,
                SpeedProfile.DEFAULT_KP, SpeedProfile.DEFAULT_POS_TOL);
    }

    /**
     * Offline run with ideal tracking ({@code s += v dt}, tau_lag = 0) until the hand-over (at
     * {@code d_go <= d_h}, or at rest within {@code d_h + 2 STOP_DIST_MARGIN}, 8.3.7e), plus the
     * hand-over tail {@code max(0, ln(d_h/posTol)/kP)} (7.10.5, 7.13.6). Guarded at
     * {@code 10 spanTime + PROFILE_TIME_GUARD} of simulated time; the guard time is returned then.
     *
     * @param kP     goal-capture gain [1/s]
     * @param posTol position tolerance [m]
     * @return time to go from sStart to within posTol of sGoal [s]
     */
    static double offlineTime(SpeedProfile profile, double jerkMax, double aAcc, double aDec,
                              double handOver, double v0, double sStart, double sGoal, double dt,
                              double kP, double posTol) {
        SpeedLaw law = new SpeedLaw(profile, jerkMax, aAcc, aDec, handOver, kP);
        double guard = 10.0 * profile.spanTime() + SpeedProfile.PROFILE_TIME_GUARD;
        double s = sStart;
        State st = new State(Math.max(0.0, v0), 0.0);
        double t = 0.0;
        while (sGoal - s > handOver
                && !(st.v() == 0.0 && t > 0.0 && sGoal - s <= handOver + 2.0 * STOP_DIST_MARGIN)) {
            double sv = Math.min(s, sGoal - handOver);
            State next = law.step(st, sv, sGoal, profile.capAt(sv), dt);
            s += 0.5 * (st.v() + next.v()) * dt;
            st = next;
            t += dt;
            if (t > guard) {
                return guard;
            }
        }
        if (handOver > posTol) {
            t += Math.log(handOver / posTol) / kP; // hand-over tail per goal (7.10.5, 8.3.7)
        }
        return t;
    }

    /** @return the junction constraints used by the last step, for tests (7.13.9 {@code cons}) */
    List<SpeedProfile.Junction> lastConstraints() {
        return lastCons;
    }

    /** @return the hand-over distance d_h [m] */
    double handOver() {
        return handOver;
    }

    /** @return the forward acceleration limit given at construction [m/s^2] */
    double aAcc() {
        return aAcc;
    }

    /** @return the braking limit given at construction [m/s^2] */
    double aDec() {
        return aDec;
    }
}
