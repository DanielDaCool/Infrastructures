package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The plan of one run (design doc sections 6, 7): heading profile, spans, junction caps, backward
 * and forward passes, the runtime speed law (7.6.5, 8.3.7) and the planned time (7.12). Arcs are
 * driven at constant speed; a withVelocity on a corner caps the whole arc. Built at command start
 * from the measured start pose and speed.
 */
final class SpeedProfile {

    /** Heading lag allowed, 5 deg [rad] (6.6, 7.4.6). */
    static final double HEADING_LAG_ALLOWED = Math.toRadians(5.0);
    /** Span merge tolerance [m] (7.3.1). */
    static final double S_EPS = 1e-6;
    /** Hand-over distance d_h [m] (8.3.7): the law reaches v_h = kP d_h here, then the 2D capture law takes over. */
    static final double HAND_OVER = 0.05;
    /** A law at rest within d_h plus this hands over too (8.3.7e) [m]. */
    static final double REST_HAND_OVER_MARGIN = 0.01;
    /** Waypoint to junction match tolerance [m]. */
    private static final double MATCH_TOL = 10.0 * S_EPS;

    /** Span kinds (7.1): a line, or a constant-speed arc. */
    enum Kind { LINE, ARC }

    // ================================================================ heading profile (6)

    /**
     * Reference heading theta(s) and theta'(s): piecewise linear through the anchors (withAngle),
     * theta stored unwrapped (6.1 to 6.4, 6.8).
     */
    static final class Heading {

        /** Two anchors closer than this along the path are one anchor; the later one wins (6.2) [m]. */
        private static final double ANCHOR_MERGE = 1e-9;

        private final double[] knotS;
        private final double[] knotTheta;

        private Heading(double[] knotS, double[] knotTheta) {
            this.knotS = knotS;
            this.knotTheta = knotTheta;
        }

        /**
         * @param path   the active (already flipped) path
         * @param theta0 measured start heading [rad], used when P_0 has no angle (6.8)
         * @return the profile
         */
        static Heading build(Path path, double theta0) {
            List<Point> points = path.points();
            double[] s = new double[points.size()];
            double[] angle = new double[points.size()];
            for (int i = 0; i < s.length; i++) {
                s[i] = path.sAtPoint(i);
                angle[i] = points.get(i).angle().map(Rotation2d::getRadians).orElse(Double.NaN);
            }
            return anchors(s, angle, theta0);
        }

        /**
         * @param pointS     s of each point the user wrote, by user index (merged points share one s)
         * @param pointAngle angle of each point [rad], NaN for a point without {@code withAngle}
         * @param theta0     measured start heading [rad], used when no anchor sits at s = 0
         * @return the profile
         */
        static Heading anchors(double[] pointS, double[] pointAngle, double theta0) {
            List<double[]> anchors = new ArrayList<>();
            for (int i = 0; i < pointS.length; i++) {
                if (!Double.isNaN(pointAngle[i])) {
                    anchors.add(new double[] {pointS[i], pointAngle[i]});
                }
            }
            anchors.sort((x, y) -> Double.compare(x[0], y[0])); // stable: the later point stays last at equal s
            List<double[]> merged = new ArrayList<>();
            for (double[] a : anchors) {
                if (!merged.isEmpty() && Math.abs(merged.get(merged.size() - 1)[0] - a[0]) <= ANCHOR_MERGE) {
                    merged.set(merged.size() - 1, a); // later one wins (6.2)
                } else {
                    merged.add(a);
                }
            }
            double thetaStart = theta0;
            int first = 0;
            if (!merged.isEmpty() && merged.get(0)[0] <= ANCHOR_MERGE) {
                thetaStart = merged.get(0)[1];
                first = 1;
            }
            int m = merged.size() - first + 1;
            double[] ks = new double[m];
            double[] kt = new double[m];
            kt[0] = thetaStart;
            for (int i = first, j = 1; i < merged.size(); i++, j++) {
                ks[j] = merged.get(i)[0];
                kt[j] = kt[j - 1] + PathGeometry.shortTurn(merged.get(i)[1] - kt[j - 1]);
            }
            return new Heading(ks, kt);
        }

        /** @return reference heading at s [rad], unwrapped (6.3); wrap it before comparing */
        double theta(double s) {
            int last = knotS.length - 1;
            if (s <= knotS[0]) {
                return knotTheta[0];
            }
            if (s >= knotS[last]) {
                return knotTheta[last];
            }
            int i = piece(s);
            return knotTheta[i] + (s - knotS[i]) / (knotS[i + 1] - knotS[i]) * (knotTheta[i + 1] - knotTheta[i]);
        }

        /** @return theta'(s) [rad/m]; 0 after the last anchor (6.4) */
        double dTheta(double s) {
            int last = knotS.length - 1;
            if (last == 0 || s >= knotS[last]) {
                return 0.0;
            }
            int i = s <= knotS[0] ? 0 : piece(s);
            return (knotTheta[i + 1] - knotTheta[i]) / (knotS[i + 1] - knotS[i]);
        }

        /** @return s of the knots (anchors), used for the kink caps (7.4.6) */
        double[] breakpointsS() {
            return knotS.clone();
        }

        /** @return heading at the end of the path (8.7.2) */
        double endHeading() {
            return knotTheta[knotTheta.length - 1];
        }

        private int piece(double s) {
            int i = Arrays.binarySearch(knotS, s);
            if (i < 0) {
                i = -i - 2;
            }
            return Math.max(0, Math.min(knotS.length - 2, i));
        }
    }

    // ================================================================ data

    private final PathConstraints c;
    private final double length;
    private final double v0;
    private final double kP;
    private final double posTol;

    // spans k = 0..K-1
    private final double[] sStart;
    private final double[] len;
    private final double[] kappa;
    private final double[] dtheta;
    private final Kind[] kind;
    private final double[] cap;
    private final double[] aA;
    private final double[] aD;

    // junctions j = 0..K
    private final double[] js;
    private final double[] jCap;
    private final double[] vB;
    private final double[] vF;
    private final boolean[] isStop;

    // sub-profiles
    private final int[] subFirst;
    private final int[] subLast;
    private final double[] subStartVB;

    private final double[] stopS; // ascending, excluding 0 and L
    private final double[] stopDwell;
    private final double endDwell;
    private double startDwell; // stop(d) on P_0: delays the start (8.6); set by build(Path, ...)
    private final double spanTime;

    // ================================================================ build

    /**
     * Builds the profile of a path (7.17). The point data the geometry does not carry (each
     * point's withVelocity and the first and last point's dwell) is read from {@code path.points()}.
     *
     * @param path        the active (already flipped) path
     * @param heading     heading profile (omega and alpha caps)
     * @param constraints resolved constraints
     * @param v0          measured start speed along T(0); negative is clamped to 0 (7.6.4); must be finite
     * @param kP          goal-capture gain k_p: hand-over speed v_h = kP d_h and the tail of profileTime (8.3.7)
     * @param posTol      position tolerance: the hand-over tail of profileTime (8.3.7)
     * @return the profile
     */
    static SpeedProfile build(Path path, Heading heading, PathConstraints constraints, double v0, double kP,
                              double posTol) {
        List<Point> points = path.points();
        int n = points.size();
        double[] wpS = new double[n];
        double[] vel = new double[n];
        for (int i = 0; i < n; i++) {
            wpS[i] = path.sAtPoint(i);
            vel[i] = points.get(i).maxVelocity().orElse(Double.NaN);
        }
        SpeedProfile profile = new SpeedProfile(path.geometry().segments(), wpS, vel, points.get(n - 1).dwellSeconds(),
                heading, constraints, v0, kP, posTol);
        for (int i = 0; i < n && wpS[i] == 0.0; i++) { // P_0 and the points merged into it: the later one wins
            if (points.get(i).dwellSeconds() > 0.0) {
                profile.startDwell = points.get(i).dwellSeconds();
            }
        }
        return profile;
    }

    private SpeedProfile(List<PathGeometry.Segment> segments, double[] waypointS, double[] pointVelocity,
                         double endDwell, Heading heading, PathConstraints constraints, double v0, double kP,
                         double posTol) {
        this.c = constraints;
        this.v0 = Math.max(0.0, v0);
        this.kP = kP;
        this.posTol = posTol;
        this.endDwell = Math.max(0.0, endDwell);
        this.length = segments.get(segments.size() - 1).sEnd();

        // Stops (7.10.1): every STOP entry strictly inside (0, L).
        List<double[]> stops = new ArrayList<>();
        for (PathGeometry.Segment g : segments) {
            if (g.type() == PathGeometry.Segment.Type.STOP && g.sStart() > S_EPS && g.sStart() < length - S_EPS) {
                stops.add(new double[] {g.sStart(), g.dwell()});
            }
        }
        stops.sort((x, y) -> Double.compare(x[0], y[0]));
        stopS = stops.stream().mapToDouble(x -> x[0]).toArray();
        stopDwell = stops.stream().mapToDouble(x -> x[1]).toArray();

        // 7.3.1 boundaries.
        List<Double> b = new ArrayList<>(List.of(0.0, length));
        segments.forEach(g -> b.add(g.sStart()));
        Arrays.stream(waypointS).forEach(b::add);
        Arrays.stream(stopS).forEach(b::add);
        Arrays.stream(heading.breakpointsS()).forEach(b::add);
        double[] sorted = b.stream().mapToDouble(Double::doubleValue)
                .map(x -> Math.max(0.0, Math.min(length, x))).sorted().toArray();
        List<Double> merged = new ArrayList<>();
        for (double x : sorted) {
            if (merged.isEmpty() || x - merged.get(merged.size() - 1) >= S_EPS) {
                merged.add(x);
            }
        }
        if (length - merged.get(merged.size() - 1) < S_EPS && merged.size() > 1) {
            merged.set(merged.size() - 1, length); // keep L exact
        } else if (merged.get(merged.size() - 1) != length) {
            merged.add(length);
        }
        int kCount = merged.size() - 1;
        js = merged.stream().mapToDouble(Double::doubleValue).toArray();

        sStart = new double[kCount];
        len = new double[kCount];
        kappa = new double[kCount];
        dtheta = new double[kCount];
        int[] segIdx = new int[kCount];
        kind = new Kind[kCount];
        for (int k = 0; k < kCount; k++) {
            sStart[k] = js[k];
            len[k] = js[k + 1] - js[k];
            double mid = 0.5 * (js[k] + js[k + 1]);
            segIdx[k] = segmentAt(segments, mid);
            PathGeometry.Segment g = segments.get(segIdx[k]);
            boolean arc = g.type() == PathGeometry.Segment.Type.ARC;
            kappa[k] = arc ? g.sigma() / g.r() : 0.0;
            kind[k] = arc ? Kind.ARC : Kind.LINE;
            dtheta[k] = heading.dTheta(mid);
        }

        // 7.4 span limits.
        cap = new double[kCount];
        aA = new double[kCount];
        aD = new double[kCount];
        for (int k = 0; k < kCount; k++) {
            double vc = c.vMax();
            double a1 = c.aAcc();
            double a2 = c.aDec();
            if (kind[k] == Kind.ARC) {
                vc = Math.min(vc, Math.sqrt(c.aLat() / Math.abs(kappa[k])));
                a1 = 0.0;
                a2 = 0.0;
            }
            double dth = Math.abs(dtheta[k]);
            if (dth > 0.0) {
                vc = Math.min(vc, c.omegaMax() / dth);
                if (a1 > 0.0) {
                    a1 = Math.min(a1, c.alphaMax() / dth);
                }
                if (a2 > 0.0) {
                    a2 = Math.min(a2, c.alphaMax() / dth);
                }
            }
            cap[k] = vc;
            aA[k] = a1;
            aD[k] = a2;
        }

        // Junction stop flags and withVelocity caps: at a line junction the junction only, on an arc the whole arc.
        isStop = new boolean[kCount + 1];
        isStop[kCount] = true;
        for (double s : stopS) {
            int j = junctionNear(s);
            if (j > 0 && j < kCount) {
                isStop[j] = true;
            }
        }
        double[] wpVel = new double[kCount + 1];
        Arrays.fill(wpVel, Double.POSITIVE_INFINITY);
        for (int i = 1; i < Math.min(waypointS.length - 1, pointVelocity.length); i++) {
            double pv = pointVelocity[i];
            if (Double.isNaN(pv) || !(pv > 0.0)) {
                continue; // none, or 0 (a stop: already a STOP entry)
            }
            int j = junctionNear(waypointS[i]);
            if (j <= 0 || j >= kCount || isStop[j]) {
                continue;
            }
            wpVel[j] = Math.min(wpVel[j], pv);
            if (kind[j - 1] == Kind.ARC && kind[j] == Kind.ARC && segIdx[j - 1] == segIdx[j]) {
                for (int k = 0; k < kCount; k++) {
                    if (segIdx[k] == segIdx[j]) {
                        cap[k] = Math.min(cap[k], pv);
                    }
                }
            }
        }

        // Junction caps (7.4) without the sub-profile start rule.
        double kinkNum = Math.sqrt(2.0 * c.alphaMax() * HEADING_LAG_ALLOWED);
        jCap = new double[kCount + 1];
        for (int j = 0; j <= kCount; j++) {
            double v = Double.POSITIVE_INFINITY;
            if (j > 0) {
                v = Math.min(v, cap[j - 1]);
            }
            if (j < kCount) {
                v = Math.min(v, cap[j]);
            }
            v = Math.min(v, wpVel[j]);
            if (j > 0 && j < kCount && !isStop[j]) {
                double jump = Math.abs(dtheta[j] - dtheta[j - 1]);
                if (jump > 1e-12) {
                    v = Math.min(v, kinkNum / jump);
                }
            }
            jCap[j] = isStop[j] ? 0.0 : v;
        }

        // Sub-profiles and passes (7.6.2, 7.10.2).
        List<Integer> bounds = new ArrayList<>(List.of(0));
        for (int j = 1; j <= kCount; j++) {
            if (isStop[j]) {
                bounds.add(j);
            }
        }
        int nSub = bounds.size() - 1;
        subFirst = new int[nSub];
        subLast = new int[nSub];
        subStartVB = new double[nSub];
        vB = new double[kCount + 1];
        vF = new double[kCount + 1];
        double total = 0.0;
        for (int p = 0; p < nSub; p++) {
            int first = bounds.get(p);
            int last = bounds.get(p + 1);
            subFirst[p] = first;
            subLast[p] = last;
            double[] lb = new double[kCount + 1];
            System.arraycopy(jCap, first, lb, first, last - first + 1);
            lb[first] = cap[first]; // normal caps at a sub-profile start, not 0 (7.10.2)
            lb[last] = 0.0;
            for (int j = last - 1; j >= first; j--) {
                lb[j] = Math.min(lb[j], Math.sqrt(lb[j + 1] * lb[j + 1] + 2.0 * aD[j] * len[j]));
            }
            double[] lf = new double[kCount + 1];
            lf[first] = Math.min(lb[first], p == 0 ? this.v0 : 0.0);
            for (int j = first + 1; j <= last; j++) {
                lf[j] = Math.min(lb[j], upSpeed(j - 1, lf[j - 1], len[j - 1]));
            }
            subStartVB[p] = lb[first];
            for (int j = first; j <= last; j++) {
                if (j > first || p == 0) {
                    vB[j] = lb[j];
                }
                vF[j] = lf[j];
            }
            for (int k = first; k < last; k++) {
                total += spanTimeOf(k, vF[k], vF[k + 1]);
            }
        }
        spanTime = total;
    }

    // ================================================================ build helpers

    /** @return index of the positive-length entry that contains s (later one at a boundary) */
    private static int segmentAt(List<PathGeometry.Segment> segments, double s) {
        int found = -1;
        for (int i = 0; i < segments.size(); i++) {
            PathGeometry.Segment g = segments.get(i);
            if (g.type() != PathGeometry.Segment.Type.STOP && g.length() > 0.0 && (found < 0 || g.sStart() <= s)) {
                found = i;
            }
        }
        return found;
    }

    private int junctionNear(double s) {
        int best = -1;
        double bestD = MATCH_TOL;
        for (int j = 0; j < js.length; j++) {
            double d = Math.abs(js[j] - s);
            if (d <= bestD) {
                best = j;
                bestD = d;
            }
        }
        return best;
    }

    /** @return speed after distance d from v1 on span k with constant aA (F_k, 7.6.2) */
    private double upSpeed(int k, double v1, double d) {
        if (aA[k] <= 0.0 || d <= 0.0) {
            return v1;
        }
        return Math.sqrt(v1 * v1 + 2.0 * aA[k] * d);
    }

    /** Closed-form span time (7.7). */
    private double spanTimeOf(int k, double vin, double vout) {
        double l = len[k];
        if (aA[k] <= 0.0 && aD[k] <= 0.0) {
            if (!(vin > 0.0)) {
                throw new IllegalStateException("SpeedProfile: the constant-speed arc span " + k + " would be entered at"
                        + " speed 0, so the robot could never cross it (an internal error, please report it with the"
                        + " path points).");
            }
            return l / vin;
        }
        double vc = cap[k];
        double brake = aD[k];
        double q1 = vc > vin ? (vc * vc - vin * vin) / (2.0 * aA[k]) : 0.0;
        double q2 = l - (vc * vc - vout * vout) / (2.0 * brake);
        if (q1 <= q2) { // cruise: the speed-up to vc and the slow-down from vc fit
            return (vc > vin ? (vc - vin) / aA[k] : 0.0) + (q2 - q1) / vc + (vc - vout) / brake;
        }
        double qs = (vout * vout - vin * vin + 2.0 * brake * l) / (2.0 * (aA[k] + brake));
        double vp = Math.max(Math.sqrt(Math.max(0.0, vin * vin + 2.0 * aA[k] * qs)), Math.max(vin, vout));
        return (vp > vin ? (vp - vin) / aA[k] : 0.0) + (vp - vout) / brake;
    }

    // ================================================================ speed law (7.6.5, 8.3.7)

    /**
     * One loop of the runtime speed law (7.6.5, 8.3.7a):
     * {@code v = min(vBack(sV), sqrt(v_h^2 + 2 D (sGoal - d_h - sV)), cap, vPrev + aAccEff(s) dt)}, with
     * {@code v_h = kP d_h} and D the smallest line aD up to the hand-over point. Braking is tied to
     * position (vBack and the hand-over envelope, limited by aD); speed-up is a time ramp, 0 on a
     * constant arc. A non-finite input, or dt &lt;= 0, holds vPrev.
     *
     * @param vPrev previous speed command [m/s]
     * @param s     progress s* (selects the speed-up limit)
     * @param sV    lag-led progress, clamped at sGoal - d_h
     * @param sGoal first stop point greater than s, or L
     * @param cap   extra cap for this loop (the correction budget, 8.3.9) [m/s]
     * @param dt    loop period [s]
     * @return the new speed command [m/s], at least 0
     */
    double speed(double vPrev, double s, double sV, double sGoal, double cap, double dt) {
        if (!Double.isFinite(vPrev) || !Double.isFinite(s) || !Double.isFinite(sV) || !Double.isFinite(sGoal)
                || !Double.isFinite(cap) || !Double.isFinite(dt) || !(dt > 0.0)) {
            return vPrev;
        }
        double sHand = sGoal - HAND_OVER;
        double vH = kP * HAND_OVER;
        double vHand = Math.sqrt(vH * vH + 2.0 * brakeLimit(sV, sHand) * Math.max(0.0, sHand - sV));
        double v = Math.min(Math.min(vBack(sV, sGoal), vHand), cap);
        return Math.max(0.0, Math.min(v, Math.max(0.0, vPrev) + accelLimit(s) * dt));
    }

    // ================================================================ queries

    /**
     * @return the planned time (7.10.5, 7.12): the closed-form span times of the passes, the dwells
     *     and the hand-over tail {@code max(0, ln(d_h/posTol)/kP)} per goal (8.3.7)
     */
    double profileTime() {
        double tail = HAND_OVER > posTol ? Math.log(HAND_OVER / posTol) / kP : 0.0;
        double t = startDwell + spanTime; // stop(d) on P_0 waits before the start (8.6)
        for (int p = 0; p < subFirst.length; p++) {
            t += tail + dwellAt(js[subLast[p]]);
        }
        return t;
    }

    /** @return the span containing s */
    int spanAt(double s) {
        int i = Arrays.binarySearch(sStart, s);
        if (i < 0) {
            i = -i - 2;
        }
        return Math.max(0, Math.min(len.length - 1, i));
    }

    /** @return vBack with s clamped into the sub-profile that ends at sGoal (at a stop the approaching robot reads 0) */
    double vBack(double s, double sGoal) {
        return vBackIn(spanInSub(s, sGoal), s);
    }

    private double vBackIn(int k, double s) {
        double q = Math.max(0.0, Math.min(len[k], s - sStart[k]));
        return Math.min(cap[k], Math.sqrt(vB[k + 1] * vB[k + 1] + 2.0 * aD[k] * (len[k] - q)));
    }

    /** @return the span at s, clamped into the sub-profile that ends at the junction nearest sGoal */
    private int spanInSub(double s, double sGoal) {
        int best = subFirst.length - 1;
        double bestD = Double.POSITIVE_INFINITY;
        for (int p = 0; p < subLast.length; p++) {
            double d = Math.abs(js[subLast[p]] - sGoal);
            if (d < bestD) {
                bestD = d;
                best = p;
            }
        }
        return Math.max(subFirst[best], Math.min(subLast[best] - 1, spanAt(s)));
    }

    /** @return speed cap vCap of the span at s (7.4) */
    double capAt(double s) {
        return cap[spanAt(s)];
    }

    /** @return the speed-up limit aAccEff in the span at s; 0 on a constant arc (7.6.5) */
    double accelLimit(double s) {
        int k = spanAt(s);
        return kind[k] == Kind.ARC ? 0.0 : aA[k];
    }

    /** @return D for the hand-over point sTo seen from sFrom (8.3.7): the smallest line aD on the way; aDec if none */
    double brakeLimit(double sFrom, double sTo) {
        int k0 = spanAt(sFrom);
        double dmin = Double.POSITIVE_INFINITY;
        for (int k = k0; k < len.length && (k == k0 || sStart[k] < sTo); k++) {
            if (kind[k] != Kind.ARC) {
                dmin = Math.min(dmin, aD[k]);
            }
        }
        return Double.isFinite(dmin) ? dmin : c.aDec();
    }

    /** @return the first stop s greater than s, or L if none: s_goal, selects the sub-profile (7.10, 8.14) */
    double nextStopS(double s) {
        for (double x : stopS) {
            if (x > s) {
                return x;
            }
        }
        return length;
    }

    /** @return dwell [s] of the stop at sStop; at L the last point's, at 0 the first point's stop(d) (7.10.4, 8.6) */
    double dwellAt(double sStop) {
        if (Math.abs(sStop - length) <= MATCH_TOL) {
            return endDwell;
        }
        if (Math.abs(sStop) <= MATCH_TOL) {
            return startDwell;
        }
        for (int i = 0; i < stopS.length; i++) {
            if (Math.abs(stopS[i] - sStop) <= MATCH_TOL) {
                return stopDwell[i];
            }
        }
        return 0.0;
    }

    /** @return path length L */
    double length() {
        return length;
    }

    /** @return hand-over distance d_h (8.3.7) */
    double handOver() {
        return HAND_OVER;
    }
}
