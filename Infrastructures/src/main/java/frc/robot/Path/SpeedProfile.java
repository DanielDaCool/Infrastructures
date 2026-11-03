package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The plan of one run (design doc sections 6, 7): heading profile, spans, junction caps, backward
 * and forward passes, room to brake after arcs, the jerk-aware online speed law (7.13) and the
 * offline planned time. Arcs are driven at constant speed; a withVelocity on a corner caps the
 * whole arc. Built at command start from the measured start pose and speed.
 */
final class SpeedProfile {

    /** Heading lag allowed, 5 deg [rad] (6.6, 7.4.6). */
    static final double HEADING_LAG_ALLOWED = Math.toRadians(5.0);
    /** Span merge tolerance [m] (7.3.1). */
    static final double S_EPS = 1e-6;
    /** Room-to-brake bisection iterations (7.8). */
    static final int ROOM_TO_BRAKE_ITER = 60;
    /** Offline profileTime guard is {@code 10 * spanTime + this} [s] (7.13.6). */
    static final double PROFILE_TIME_GUARD = 10.0;
    /** Margin added to the stopping distance [m] (7.13.4). */
    static final double STOP_DIST_MARGIN = 0.005;
    /** Waypoint to junction match tolerance [m]. */
    private static final double MATCH_TOL = 10.0 * S_EPS;
    /** Cap comparison tolerance (rounding) [m/s]. */
    private static final double CAP_TOL = 1e-9;

    /** Span kinds (7.1): a line, or a constant-speed arc. */
    enum Kind { LINE, ARC }

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
     * @param guardHit true if the run hit the guard {@code 10 spanTime + PROFILE_TIME_GUARD} (a bug)
     */
    record OfflineResult(double time, boolean guardHit) {
    }

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
            if (!Double.isFinite(theta0)) {
                throw new IllegalArgumentException("SpeedProfile.Heading: the measured start heading must be a finite"
                        + " number (radians), but it is " + theta0 + ". Check that the pose supplier returns a real"
                        + " rotation, for example swerve::getPose.");
            }
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
    private final double handOver;
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
    private double profileTimeCache = Double.NaN;
    private boolean profileGuardHit;

    // speed law: constraint list of the current step, preallocated (junctions ahead plus the hand-over point)
    private final double[] consS;
    private final double[] consVt;
    private final double[] consD;
    private final boolean[] consEntry;
    private int consCount;

    // ================================================================ build

    /**
     * Builds the profile of a path (7.17). The point data the geometry does not carry (each
     * point's withVelocity and the first and last point's dwell) is read from {@code path.points()}.
     *
     * @param path        the active (already flipped) path
     * @param heading     heading profile (omega and alpha caps)
     * @param constraints resolved constraints
     * @param v0          measured start speed along T(0); negative is clamped to 0 (7.6.4); must be finite
     * @param kP          goal-capture gain k_p: hand-over d_h and v_h = kP d_h (8.3.7)
     * @param posTol      position tolerance: the hand-over tail of profileTime (7.13.6)
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
            OptionalDouble mv = points.get(i).maxVelocity();
            vel[i] = mv.isPresent() ? mv.getAsDouble() : Double.NaN;
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

    /**
     * Core builder from plain geometry data (7.17); tests call it with hand-built entries.
     *
     * @param segments      LINE / ARC / STOP entries in order
     * @param waypointS     s of each user point, by user index
     * @param pointVelocity withVelocity per user point (NaN = none), same length as waypointS
     * @param endDwell      dwell at the last point [s]
     * @param heading       heading profile
     * @param constraints   resolved constraints
     * @param v0            start speed (finite; negative is clamped to 0)
     * @param kP            goal-capture gain (hand-over speed v_h = kP d_h, 8.3.7)
     * @param posTol        position tolerance for the hand-over tail (7.13.6)
     * @return the profile
     */
    static SpeedProfile build(List<PathGeometry.Segment> segments, double[] waypointS, double[] pointVelocity,
                              double endDwell, Heading heading, PathConstraints constraints, double v0, double kP,
                              double posTol) {
        return new SpeedProfile(segments, waypointS, pointVelocity, endDwell, heading, constraints, v0, kP, posTol);
    }

    /** @return the hand-over distance d_h = min(D_H_MAX, jerkMax dt_nom / kP^2) (8.3.7) */
    static double handOverDistance(double jerkMax, double kP) {
        return Math.min(FollowPathCommand.D_H_MAX, jerkMax * FollowPathCommand.DT_NOM / (kP * kP));
    }

    private SpeedProfile(List<PathGeometry.Segment> segments, double[] waypointS, double[] pointVelocity,
                         double endDwell, Heading heading, PathConstraints constraints, double v0, double kP,
                         double posTol) {
        if (!Double.isFinite(v0) || !Double.isFinite(endDwell)) {
            throw new IllegalArgumentException("SpeedProfile: the start speed (" + v0 + " m/s) and the end wait ("
                    + endDwell + " s) must be finite numbers. Check that the pose and speed suppliers never return NaN"
                    + " or infinity.");
        }
        if (!(kP > 0.0) || !Double.isFinite(kP) || !(posTol > 0.0) || !Double.isFinite(posTol)) {
            throw new IllegalArgumentException("SpeedProfile: kP is " + kP + " and posTol is " + posTol + " m; both must be"
                    + " finite numbers above 0. Fix them in the PathFollowerConfig, for example"
                    + " PathFollowerConfig.defaults().withKP(3.0).withPosTol(0.03).");
        }
        this.c = constraints;
        this.v0 = Math.max(0.0, v0);
        this.kP = kP;
        this.posTol = posTol;
        this.handOver = handOverDistance(constraints.jerkMax(), kP);
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
                if (kind[j] == Kind.LINE && j > first && kind[j - 1] == Kind.ARC) { // line after an arc
                    lb[j] = Math.min(lb[j], vJerk(j, last, lb));
                }
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

        int n = kCount + 2;
        consS = new double[n];
        consVt = new double[n];
        consD = new double[n];
        consEntry = new boolean[n];
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

    /** Room to brake after an arc (7.8): largest v at junction j, a = 0, that meets every later constraint. */
    private double vJerk(int j, int last, double[] lb) {
        int n = last - j;
        double[] dist = new double[n];
        double[] vt = new double[n];
        double[] dq = new double[n];
        double dmin = Double.POSITIVE_INFINITY;
        for (int q = j + 1, i = 0; q <= last; q++, i++) {
            if (kind[q - 1] != Kind.ARC) {
                dmin = Math.min(dmin, aD[q - 1]);
            }
            dq[i] = Double.isFinite(dmin) ? dmin : c.aDec();
            dist[i] = q == last ? js[q] - handOver - js[j] : js[q] - js[j];
            vt[i] = q == last ? kP * handOver : lb[q];
        }
        double hi = lb[j];
        if (roomOk(hi, dist, vt, dq)) {
            return hi;
        }
        double lo = 0.0;
        for (int it = 0; it < ROOM_TO_BRAKE_ITER; it++) {
            double mid = 0.5 * (lo + hi);
            if (roomOk(mid, dist, vt, dq)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    private boolean roomOk(double v, double[] dist, double[] vt, double[] dq) {
        for (int i = 0; i < dist.length; i++) {
            if (v > vt[i] && stopDist(v, 0.0, vt[i], c.jerkMax(), dq[i]) + STOP_DIST_MARGIN > dist[i]) {
                return false;
            }
        }
        return true;
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

    // ================================================================ speed law (7.13)

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
        double jerkMax = c.jerkMax();
        double v = state.v();
        double a = state.a();
        double jdt = jerkMax * dt;

        int n = fillJunctionsAhead(s, sGoal);
        double sHand = sGoal - handOver;
        consS[n] = sHand;
        consVt[n] = kP * handOver;
        consD[n] = brakeLimit(s, sHand);
        consEntry[n] = true;
        consCount = n + 1;

        boolean onArc = onConstantArc(s);
        double aDcur = decelAt(s);
        double failedD = Double.NEGATIVE_INFINITY;

        int nCand = onArc ? 2 : 3;
        for (int cand = 0; cand < nCand; cand++) {
            boolean up = !onArc && cand == 0;
            double a1;
            if (up) {
                a1 = Math.min(a + jdt, accelLimit(s));
            } else if ((onArc && cand == 0) || (!onArc && cand == 1)) {
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
            boolean ok = !(v1 > cap + CAP_TOL && a1 >= 0.0)
                    && !(a1 > 0.0 && v1 + a1 * a1 / (2.0 * jerkMax) > cap + CAP_TOL);
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

        double dBrake = Math.min(failedD == Double.NEGATIVE_INFINITY ? consD[0] : failedD, aDcur);
        double a1 = Math.min(a + jdt, Math.max(Math.max(a - jdt, -dBrake), -Math.sqrt(2.0 * jerkMax * Math.max(v, 0.0))));
        a1 = Math.max(a1, -aDcur); // traction clamp on the brake candidate too (traction before jerk)
        double v1 = v + 0.5 * (a + a1) * dt;
        if (v1 <= 0.0) {
            v1 = 0.0;
            a1 = Math.max(a1, 0.0); // speed floor
        }
        return new State(v1, a1);
    }

    /** Writes the junctions with s in (sFrom, sGoal), ascending, into the law's constraint list. @return how many */
    private int fillJunctionsAhead(double sFrom, double sGoal) {
        int n = 0;
        double dmin = Double.POSITIVE_INFINITY;
        for (int j = spanAt(sFrom) + 1; j < js.length; j++) {
            if (kind[j - 1] != Kind.ARC) {
                dmin = Math.min(dmin, aD[j - 1]);
            }
            if (js[j] >= sGoal) {
                break;
            }
            if (js[j] <= sFrom) {
                continue;
            }
            consS[n] = js[j];
            consVt[n] = vB[j];
            consD[n] = Double.isFinite(dmin) ? dmin : c.aDec();
            consEntry[n] = j < len.length && kind[j - 1] == Kind.LINE && kind[j] != Kind.LINE;
            n++;
        }
        return n;
    }

    /** @return the junction constraints used by the last step (7.13.9 {@code cons}), for tests */
    List<Junction> lastConstraints() {
        List<Junction> out = new ArrayList<>(consCount);
        for (int i = 0; i < consCount; i++) {
            out.add(new Junction(consS[i], consVt[i], consD[i], consEntry[i]));
        }
        return out;
    }

    /**
     * Distance needed to go from (v, a) to speed vt with jerk jerkMax and braking limit D (7.13.3).
     * Allocation-free; runs every loop.
     *
     * @return stopping distance [m]
     */
    static double stopDist(double v, double a, double vt, double jerkMax, double decel) {
        if (!(jerkMax > 0.0) || !(decel > 0.0)) {
            throw new IllegalArgumentException("SpeedProfile.stopDist: jerkMax is " + jerkMax + " m/s^3 and the braking"
                    + " limit is " + decel + " m/s^2; both must be above 0. Use positive limits, for example"
                    + " PathConstraints.fromConfig(config).withJerkMax(50.0).");
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
            double t1 = (b - PathGeometry.vsqrt(b * b - 2.0 * jerkMax * (v - vt))) / jerkMax;
            return d + v * t1 - b * t1 * t1 / 2.0 + jerkMax * t1 * t1 * t1 / 6.0;
        }
        double vv = v + b * b / (2.0 * jerkMax); // virtual start
        double dv = vv - vt;
        double ds;
        if (dv >= decel * decel / jerkMax) { // full D
            ds = (vv * vv - vt * vt) / (2.0 * decel) + (vv + vt) * decel / (2.0 * jerkMax);
        } else { // triangular
            ds = (vv + vt) * PathGeometry.vsqrt(dv / jerkMax);
        }
        double tr = b / jerkMax;
        return d + ds - (vv * tr - b * b * b / (6.0 * jerkMax * jerkMax));
    }

    /**
     * Offline run with ideal tracking ({@code s += v dt}, tau_lag = 0) until the hand-over (at
     * {@code d_go <= d_h}, or at rest within {@code d_h + 2 STOP_DIST_MARGIN}, 8.3.7e), plus the
     * hand-over tail {@code max(0, ln(d_h/posTol)/kP)} (7.10.5, 7.13.6). Guarded at
     * {@code 10 spanTime + PROFILE_TIME_GUARD} of simulated time; then the guard time is returned.
     *
     * @param v0     start speed [m/s]
     * @param sFrom  start s
     * @param sGoal  end s (a stop or L)
     * @param dt     step [s]
     * @return the time and the guard flag
     */
    OfflineResult offlineRun(double v0, double sFrom, double sGoal, double dt) {
        double guard = 10.0 * spanTime + PROFILE_TIME_GUARD;
        double s = sFrom;
        State st = new State(Math.max(0.0, v0), 0.0);
        double t = 0.0;
        while (sGoal - s > handOver && !(st.v() == 0.0 && t > 0.0 && sGoal - s <= handOver + 2.0 * STOP_DIST_MARGIN)) {
            double sv = Math.min(s, sGoal - handOver);
            State next = step(st, sv, sGoal, capAt(sv), dt);
            s += 0.5 * (st.v() + next.v()) * dt;
            st = next;
            t += dt;
            if (t > guard) {
                return new OfflineResult(guard, true);
            }
        }
        if (handOver > posTol) {
            t += Math.log(handOver / posTol) / kP; // hand-over tail per goal
        }
        return new OfflineResult(t, false);
    }

    // ================================================================ queries

    /** @return the planned time including dwells, from the offline jerk law (7.12, 7.13.6, 7.10.5) */
    double profileTime() {
        if (Double.isNaN(profileTimeCache)) {
            double t = startDwell; // stop(d) on P_0 waits before the start (8.6)
            boolean hit = false;
            for (int p = 0; p < subFirst.length; p++) {
                double s1 = js[subLast[p]];
                OfflineResult r = offlineRun(p == 0 ? v0 : 0.0, js[subFirst[p]], s1, FollowPathCommand.DT_NOM);
                t += r.time() + dwellAt(s1);
                hit |= r.guardHit();
            }
            profileGuardHit = hit;
            profileTimeCache = t;
        }
        return profileTimeCache;
    }

    /** @return true if the offline run of {@link #profileTime()} hit its guard (a bug; the guard time is used) */
    boolean profileGuardHit() {
        profileTime();
        return profileGuardHit;
    }

    /** @return the span containing s (7.13.10 step 1) */
    int spanAt(double s) {
        int i = Arrays.binarySearch(sStart, s);
        if (i < 0) {
            i = -i - 2;
        }
        return Math.max(0, Math.min(len.length - 1, i));
    }

    /**
     * @return backward-pass speed limit at s, {@code min(vCap, sqrt(vB[k+1]^2 + 2 aD (l - q)))} (7.13.10). At a
     *     junction it reads the later span: at a stop that is the restart envelope of the next sub-profile.
     */
    double vBack(double s) {
        return vBackIn(spanAt(s), s);
    }

    /** @return vBack with s clamped into the sub-profile that ends at sGoal (at a stop the approaching robot reads 0) */
    double vBack(double s, double sGoal) {
        return vBackIn(spanInSub(s, sGoal), s);
    }

    /** @return full profile speed at s, for logging (7.13.10 step 4); later span at a junction */
    double vProf(double s) {
        return vProfIn(spanAt(s), s);
    }

    /** @return vProf with s clamped into the sub-profile that ends at sGoal */
    double vProf(double s, double sGoal) {
        return vProfIn(spanInSub(s, sGoal), s);
    }

    private double vBackIn(int k, double s) {
        double q = Math.max(0.0, Math.min(len[k], s - sStart[k]));
        return Math.min(cap[k], Math.sqrt(vB[k + 1] * vB[k + 1] + 2.0 * aD[k] * (len[k] - q)));
    }

    private double vProfIn(int k, double s) {
        double q = Math.max(0.0, Math.min(len[k], s - sStart[k]));
        double up = upSpeed(k, vF[k], q);
        return Math.min(cap[k], Math.min(up, Math.sqrt(vF[k + 1] * vF[k + 1] + 2.0 * aD[k] * (len[k] - q))));
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

    /** @return braking limit at s: the span's aD on a line, aDec on a constant arc (7.13.9) */
    double decelAt(double s) {
        int k = spanAt(s);
        return kind[k] == Kind.ARC ? c.aDec() : aD[k];
    }

    /** @return the acceleration limit in the span at s; 0 on a constant arc (7.13.9) */
    double accelLimit(double s) {
        int k = spanAt(s);
        return kind[k] == Kind.ARC ? 0.0 : aA[k];
    }

    /** @return true if s is on a constant-speed arc, where the law has no "up" candidate (7.13.9) */
    boolean onConstantArc(double s) {
        return kind[spanAt(s)] == Kind.ARC;
    }

    /** @return D for a constraint at sTo seen from sFrom (7.13.4): the smallest line aD on the way; aDec if none */
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

    /** @return closed-form motion time of the span profile, the no-jerk lower bound without dwells (7.12) */
    double spanTime() {
        return spanTime;
    }

    /** @return path length L */
    double length() {
        return length;
    }

    /** @return the resolved constraints */
    PathConstraints constraints() {
        return c;
    }

    /** @return goal-capture gain k_p of the profile (8.3.7) */
    double kP() {
        return kP;
    }

    /** @return hand-over distance d_h used by the profile (8.3.7) */
    double handOver() {
        return handOver;
    }

    /** @return number of spans (7.3) */
    int spanCount() {
        return len.length;
    }

    /** @return s at the start of span k */
    double spanStart(int k) {
        return sStart[k];
    }

    /** @return length of span k */
    double spanLength(int k) {
        return len[k];
    }

    /** @return kind of span k */
    Kind spanKind(int k) {
        return kind[k];
    }

    /** @return cap of span k */
    double spanCap(int k) {
        return cap[k];
    }

    /** @return speed-up limit of span k */
    double spanAccel(int k) {
        return aA[k];
    }

    /** @return slow-down limit of span k */
    double spanDecel(int k) {
        return aD[k];
    }

    /** @return curvature of span k */
    double spanKappa(int k) {
        return kappa[k];
    }

    /** @return theta' of span k */
    double spanDTheta(int k) {
        return dtheta[k];
    }

    /** @return s of junction j */
    double junctionS(int j) {
        return js[j];
    }

    /** @return junction cap J[j] (7.4) */
    double junctionCap(int j) {
        return jCap[j];
    }

    /** @return vB[j]; 0 at a stop (the end of the sub-profile before it) */
    double junctionVB(int j) {
        return vB[j];
    }

    /** @return vF[j] */
    double junctionVF(int j) {
        return vF[j];
    }

    /** @return number of sub-profiles */
    int subProfileCount() {
        return subFirst.length;
    }

    /** @return vB at the first junction of sub-profile p (its restart envelope, 7.10.2) */
    double subProfileStartVB(int p) {
        return subStartVB[p];
    }
}
