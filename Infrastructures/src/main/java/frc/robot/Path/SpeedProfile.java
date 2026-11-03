package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The plan of one run (design doc sections 6, 7): the heading profile, a short list of speed limits, and the speed
 * law computed every loop (like the 2025 getVelocity). The law brakes toward every limit ahead of the robot and
 * before the current goal (the next stop or the end). Built at command start from the measured start pose and speed.
 */
final class SpeedProfile {

    /** Hand-over distance d_h [m] (8.3.7): the law reaches v_h = kP d_h here, then the 2D capture law takes over. */
    static final double HAND_OVER = 0.05;
    /** A law at rest within d_h plus this hands over too (8.3.7e) [m]. */
    static final double REST_HAND_OVER_MARGIN = 0.01;
    /** Loop period of the planned-time simulation [s]. */
    static final double SIM_DT = 0.02;
    private static final double S_EPS = 1e-6;

    /** Reference heading theta(s): linear between the withAngle anchors (shortest turn), unwrapped, last held (6). */
    record Heading(double[] knotS, double[] knotTheta) {

        /** @return the profile of the active path; theta0 (measured) starts it when P_0 has no angle (6.8) */
        static Heading build(Path path, double theta0) {
            List<Point> points = path.points();
            double[] s = new double[points.size()];
            double[] angle = new double[s.length];
            for (int i = 0; i < s.length; i++) {
                s[i] = path.sAtPoint(i);
                angle[i] = points.get(i).angle().map(Rotation2d::getRadians).orElse(Double.NaN);
            }
            return anchors(s, angle, theta0);
        }

        /** @return the profile from point s and angles [rad] (NaN: no angle); at equal s the later point wins */
        static Heading anchors(double[] pointS, double[] pointAngle, double theta0) {
            List<double[]> a = new ArrayList<>(List.of(new double[] {0.0, theta0}));
            for (int i = 0; i < pointS.length; i++) {
                if (!Double.isNaN(pointAngle[i])) {
                    if (pointS[i] - a.get(a.size() - 1)[0] <= 1e-9) {
                        a.remove(a.size() - 1);
                    }
                    double last = a.isEmpty() ? pointAngle[i] : a.get(a.size() - 1)[1];
                    a.add(new double[] {pointS[i], last + PathGeometry.shortTurn(pointAngle[i] - last)});
                }
            }
            return new Heading(a.stream().mapToDouble(x -> x[0]).toArray(), a.stream().mapToDouble(x -> x[1]).toArray());
        }

        /** @return reference heading at s [rad], unwrapped (6.3) */
        double theta(double s) {
            int i = piece(s);
            return i >= 0 ? knotTheta[i] + (s - knotS[i]) * dTheta(s) : s <= knotS[0] ? knotTheta[0] : endHeading();
        }

        /** @return theta'(s) [rad/m]; 0 before the first and after the last anchor (6.4) */
        double dTheta(double s) {
            int i = piece(s);
            return i < 0 ? 0.0 : (knotTheta[i + 1] - knotTheta[i]) / (knotS[i + 1] - knotS[i]);
        }

        /** @return s of the anchors */
        double[] breakpointsS() { return knotS.clone(); }

        /** @return heading at the end of the path (8.7.2) */
        double endHeading() { return knotTheta[knotTheta.length - 1]; }

        /** @return the piece i with knotS[i] &lt;= s &lt; knotS[i + 1], or -1 outside the anchors */
        private int piece(double s) {
            int i = Arrays.binarySearch(knotS, s);
            i = i < 0 ? -i - 2 : i;
            return i >= 0 && i < knotS.length - 1 ? i : -1;
        }
    }

    /**
     * A speed limit vCap on [sStart, sEnd] (a point limit has sStart == sEnd). vGrip = sqrt(aLat r) on an arc (speed-up
     * shares the grip), rate = alphaMax/|theta'| on a heading piece (speed-up and braking toward it); else infinite.
     */
    record Limit(double sStart, double sEnd, double vCap, double vGrip, double rate) {
        Limit(double sStart, double sEnd, double vCap) {
            this(sStart, sEnd, vCap, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
    }

    private final List<Limit> limits = new ArrayList<>();
    private final List<double[]> stops = new ArrayList<>(); // {s, dwell} inside (0, L), ascending
    private PathConstraints c;
    private double kP, length, startDwell, endDwell, profileTime;

    /**
     * Builds the limits (arcs: sqrt(aLat r), capped by a withVelocity corner; heading pieces: omegaMax/|theta'|;
     * withVelocity points off arcs) and simulates the law once with ideal tracking for profileTime.
     *
     * @param v0     measured start speed along T(0); negative is clamped to 0 (7.6.4)
     * @param kP     goal-capture gain: hand-over speed v_h = kP d_h (8.3.7)
     * @param posTol position tolerance: hand-over tail {@code max(0, ln(d_h/posTol)/kP)} per goal
     */
    static SpeedProfile build(Path path, Heading heading, PathConstraints constraints, double v0, double kP,
                              double posTol) {
        List<PathGeometry.Segment> segs = path.geometry().segments();
        SpeedProfile p = new SpeedProfile();
        p.c = constraints;
        p.kP = kP;
        p.length = segs.get(segs.size() - 1).sEnd();
        List<Point> points = path.points();
        int n = points.size();
        for (PathGeometry.Segment g : segs) {
            if (g.type() == PathGeometry.Segment.Type.ARC) {
                double vGrip = Math.sqrt(constraints.aLat() * g.r());
                p.limits.add(new Limit(g.sStart(), g.sEnd(), vGrip, vGrip, Double.POSITIVE_INFINITY));
            } else if (g.type() == PathGeometry.Segment.Type.STOP && g.sStart() > S_EPS && g.sStart() < p.length - S_EPS) {
                p.stops.add(new double[] {g.sStart(), g.dwell()});
            }
        }
        p.stops.sort((x, y) -> Double.compare(x[0], y[0]));
        for (int i = 1; i < n - 1; i++) { // withVelocity: the whole arc of its corner, else a point limit
            double s = path.sAtPoint(i);
            double v = points.get(i).maxVelocity().orElse(0.0);
            Limit arc = p.limits.stream().filter(l -> s > l.sStart() + S_EPS && s < l.sEnd() - S_EPS).findFirst().orElse(null);
            if (v > 0.0) {
                p.limits.remove(arc);
                p.limits.add(arc == null ? new Limit(s, s, v)
                        : new Limit(arc.sStart(), arc.sEnd(), Math.min(v, arc.vCap()), arc.vGrip(), arc.rate()));
            }
        }
        double[] k = heading.breakpointsS();
        for (int i = 0; i + 1 < k.length; i++) {
            double dth = Math.abs(heading.dTheta(0.5 * (k[i] + k[i + 1])));
            if (dth > 0.0) {
                p.limits.add(new Limit(k[i], k[i + 1], constraints.omegaMax() / dth, Double.POSITIVE_INFINITY,
                        constraints.alphaMax() / dth));
            }
        }
        for (int i = 0; i < n && path.sAtPoint(i) == 0.0; i++) { // stop(d) on P_0 and the points merged into it
            p.startDwell = points.get(i).dwellSeconds() > 0.0 ? points.get(i).dwellSeconds() : p.startDwell;
        }
        p.endDwell = Math.max(0.0, points.get(n - 1).dwellSeconds());
        double tail = HAND_OVER > posTol ? Math.log(HAND_OVER / posTol) / kP : 0.0;
        double s = 0.0;
        double v = Math.max(0.0, v0);
        double t = p.startDwell;
        double dwells = p.startDwell + p.endDwell + p.stops.stream().mapToDouble(x -> x[1]).sum();
        double vSlow = p.limits.stream().mapToDouble(Limit::vCap).reduce(constraints.vMax(), Math::min);
        double tMax = 10.0 * (p.length / vSlow + dwells) + 10.0; // far longer than any real run
        for (double sGoal = p.nextStopS(0.0); ; ) {
            if (t > tMax || !Double.isFinite(v)) {
                throw new IllegalStateException("SpeedProfile: the planned run does not reach s = " + sGoal + " m (stuck at s = "
                        + s + " m); an internal error, please report it with the path points and the constraints.");
            }
            if (sGoal - s <= HAND_OVER) { // hand-over: the tail and the dwell, then the next goal from rest
                t += tail + p.dwellAt(sGoal);
                if (sGoal >= p.length) {
                    break;
                }
                s = sGoal;
                sGoal = p.nextStopS(s);
                v = 0.0;
            } else {
                double next = p.speed(v, s, Math.min(s, sGoal - HAND_OVER), sGoal, Double.POSITIVE_INFINITY, SIM_DT);
                s += 0.5 * (v + next) * SIM_DT;
                v = next;
                t += SIM_DT;
            }
        }
        p.profileTime = t;
        return p;
    }

    /**
     * One loop of the speed law (7.6.5, 8.3.7a): the smallest of {@link #ceiling}, cap (the correction budget, 8.3.9)
     * and {@code vPrev + aAcc dt}. A non-finite input, or dt &lt;= 0, holds vPrev.
     *
     * @param s     progress s*: limits overlapping [s*, sV] apply
     * @param sV    lag-led progress {@code s* + v_prev TAU_LAG}, clamped at sGoal - d_h
     * @param sGoal first stop point greater than s, or L
     * @return the new speed command [m/s], at least 0
     */
    double speed(double vPrev, double s, double sV, double sGoal, double cap, double dt) {
        if (!Double.isFinite(vPrev) || !Double.isFinite(s) || !Double.isFinite(sV) || !Double.isFinite(sGoal)
                || Double.isNaN(cap) || !Double.isFinite(dt) || !(dt > 0.0)) {
            return vPrev;
        }
        double vp = Math.max(0.0, vPrev);
        double up = c.aAcc(); // speed-up: shares the grip on an arc, alphaMax/|theta'| on a heading piece
        for (Limit l : limits) {
            if (l.sStart() <= sV && l.sEnd() >= s) {
                double lat = vp * vp / (l.vGrip() * l.vGrip());
                up = Math.min(up, Math.min(l.rate(), c.aAcc() * Math.sqrt(Math.max(0.0, 1.0 - lat * lat))));
            }
        }
        double v = Math.min(Math.min(cap, ceiling(s, sV, sGoal, dt)), vp + up * dt);
        if (v > vp) { // speeding up moves the lead sV by TAU_LAG dv more next loop: brake for that now
            v = Math.max(vp, Math.min(v, ceiling(s, sV + PathFollowerConfig.TAU_LAG * (v - vp), sGoal, dt)));
        }
        return Math.max(0.0, v);
    }

    /**
     * @return the smallest of vMax, the limits overlapping [s, sV], {@code sqrt(vCap^2 + 2 d (sStart - sV))} of every
     *     limit ahead of sV and before sGoal, and the hand-over envelope {@code sqrt(v_h^2 + 2 aDec (sGoal - d_h - sV))};
     *     d = min(aDec, rate); braking terms in the discrete form, so one loop never brakes harder than d
     */
    private double ceiling(double s, double sV, double sGoal, double dt) {
        double d = c.aDec();
        double vH = kP * HAND_OVER;
        double v = Math.min(c.vMax(),
                discrete(Math.sqrt(vH * vH + 2.0 * d * dt * vH + 2.0 * d * Math.max(0.0, sGoal - HAND_OVER - sV)), d, dt));
        for (Limit l : limits) {
            if (l.sStart() <= sV && l.sEnd() >= s) {
                v = Math.min(v, l.vCap());
            } else if (l.sStart() > sV && l.sStart() < sGoal) {
                double dl = Math.min(d, l.rate());
                v = Math.min(v, discrete(Math.sqrt(l.vCap() * l.vCap() + 2.0 * dl * (l.sStart() - sV)), dl, dt));
            }
        }
        return v;
    }

    /** @return the discrete braking form of the envelope speed vE: v with v^2 + 2 d dt v = vE^2 */
    private static double discrete(double vE, double d, double dt) {
        return Math.sqrt(vE * vE + d * dt * d * dt) - d * dt;
    }

    /** @return the planned time [s] (7.12): start dwell, the simulated run, a hand-over tail and a dwell per goal */
    double profileTime() { return profileTime; }

    /** @return the speed limits */
    List<Limit> limits() { return limits; }

    /** @return the first stop s greater than s, or L if none (s_goal) */
    double nextStopS(double s) {
        return stops.stream().mapToDouble(x -> x[0]).filter(x -> x > s).findFirst().orElse(length);
    }

    /** @return dwell [s] of the stop at sStop; at L the last point's, at 0 the first point's stop(d) (8.6) */
    double dwellAt(double sStop) {
        return Math.abs(sStop - length) <= 1e-5 ? endDwell : Math.abs(sStop) <= 1e-5 ? startDwell
                : stops.stream().filter(x -> Math.abs(x[0] - sStop) <= 1e-5).mapToDouble(x -> x[1]).findFirst().orElse(0.0);
    }

    /** @return path length L */
    double length() { return length; }

    /** @return hand-over distance d_h (8.3.7) */
    double handOver() { return HAND_OVER; }
}
