package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;

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

    /** Nominal loop period of the offline profileTime run [s] (7.13.6). */
    static final double OFFLINE_DT = Tracker.DT_NOM;

    /** Span kinds (7.1). */
    enum Kind { LINE, ARC, ARC_VAR }

    /** Samples per face-target sub-span for theta'_max and theta''_max (6.9). */
    private static final int FT_SAMPLES = 20;
    /** Bisection iterations of the closed-form span time with the motor curve. */
    private static final int PEAK_ITER = 100;
    /** Waypoint to junction match tolerance [m]. */
    private static final double MATCH_TOL = 10.0 * S_EPS;

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

    // ---------------------------------------------------------------- data
    private final PathConstraints c;
    private final double length;
    private final double v0;
    private final double handOver;
    private final double kP;
    private final double posTol;
    private final boolean motor;
    private final double motorA0;
    private final double motorVFree;
    private final boolean faceMode;

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

    // ---------------------------------------------------------------- build

    /**
     * Builds the profile of a path (7.17). The point data the segment table does not carry (each
     * point's withVelocity and the last point's dwell) is read from {@code path.points()}.
     *
     * @param path         the active (already flipped) path; uses table(), points() and sAtPoint
     * @param heading      heading profile (omega and alpha caps)
     * @param constraints  resolved constraints
     * @param kinematics   motor curve (7.9) and wheel cap; may be null to skip both
     * @param moduleRadius R_mod for the wheel-speed cap, 0 = skip (7.11)
     * @param v0           measured start speed along T(0); negative is clamped to 0 (7.6.4); must be finite
     * @param kP           goal-capture gain k_p: hand-over d_h and v_h = kP d_h (8.3.7)
     * @param posTol       position tolerance: the hand-over tail of profileTime (7.13.6)
     * @return the profile
     */
    static SpeedProfile build(Path path, HeadingProfile heading, PathConstraints constraints,
                              KinematicsConfig kinematics, double moduleRadius, double v0, double kP,
                              double posTol) {
        if (path == null) {
            throw new IllegalArgumentException("SpeedProfile: path must not be null");
        }
        List<Point> points = path.points();
        int n = points.size();
        double[] wpS = new double[n];
        double[] vel = new double[n];
        for (int i = 0; i < n; i++) {
            wpS[i] = path.sAtPoint(i);
            OptionalDouble mv = points.get(i).maxVelocity();
            vel[i] = mv.isPresent() ? mv.getAsDouble() : Double.NaN;
        }
        double endDwell = points.get(n - 1).dwellSeconds();
        double startDwell = 0.0;
        for (int i = 0; i < n && wpS[i] == 0.0; i++) { // P_0 and the points merged into it: the later one wins
            if (points.get(i).dwellSeconds() > 0.0) {
                startDwell = points.get(i).dwellSeconds();
            }
        }
        SpeedProfile profile = build(HeadingProfile.segmentsOf(path.table()), wpS, vel, endDwell, heading, constraints,
                kinematics, moduleRadius, v0, kP, posTol);
        profile.startDwell = startDwell;
        return profile;
    }

    /**
     * Core builder from plain geometry data (7.17); {@link #build(Path, HeadingProfile, PathConstraints,
     * KinematicsConfig, double, double, double, double)} calls it, tests call it with hand-built entries.
     * Point data required: pointVelocity must have one entry per user point (NaN = no withVelocity).
     *
     * @param segments      LINE / ARC / STOP entries in order
     * @param waypointS     s of each user point, by user index
     * @param pointVelocity withVelocity per user point (NaN = none); required, same length as waypointS
     * @param endDwell      dwell at the last point [s]
     * @param heading       heading profile
     * @param constraints   resolved constraints
     * @param kinematics    motor curve and wheel cap source, or null
     * @param moduleRadius  R_mod, 0 = no wheel cap
     * @param v0            start speed (finite; negative is clamped to 0)
     * @param kP            goal-capture gain (hand-over speed v_h = kP d_h, 8.3.7)
     * @param posTol        position tolerance for the hand-over tail (7.13.6)
     * @return the profile
     */
    static SpeedProfile build(List<Segment> segments, double[] waypointS, double[] pointVelocity, double endDwell,
                              HeadingProfile heading, PathConstraints constraints, KinematicsConfig kinematics,
                              double moduleRadius, double v0, double kP, double posTol) {
        return new SpeedProfile(segments, waypointS, pointVelocity, endDwell, heading, constraints, kinematics,
                moduleRadius, v0, kP, posTol);
    }

    /** @return the hand-over distance d_h = min(D_H_MAX, jerkMax dt_nom / kP^2) (8.3.7) */
    static double handOverDistance(double jerkMax, double kP) {
        return Math.min(Tracker.D_H_MAX, jerkMax * Tracker.DT_NOM / (kP * kP));
    }

    private SpeedProfile(List<Segment> segments, double[] waypointS, double[] pointVelocity, double endDwell,
                         HeadingProfile heading, PathConstraints constraints, KinematicsConfig kinematics,
                         double moduleRadius, double v0, double kP, double posTol) {
        if (constraints == null || heading == null || segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException("SpeedProfile: segments, heading and constraints are required");
        }
        if (waypointS == null || pointVelocity == null || pointVelocity.length != waypointS.length) {
            throw new IllegalArgumentException(
                    "SpeedProfile: point data required: one pointVelocity entry (NaN = none) per waypoint");
        }
        requireFinite("v0", v0);
        requireFinite("endDwell", endDwell);
        requireFinite("moduleRadius", moduleRadius);
        if (!(kP > 0.0) || !Double.isFinite(kP) || !(posTol > 0.0) || !Double.isFinite(posTol)) {
            throw new IllegalArgumentException("SpeedProfile: kP and posTol must be finite and > 0");
        }
        this.c = constraints;
        this.v0 = Math.max(0.0, v0);
        this.kP = kP;
        this.posTol = posTol;
        this.handOver = handOverDistance(constraints.jerkMax(), kP);
        this.endDwell = Math.max(0.0, endDwell);
        this.motor = kinematics != null;
        this.motorA0 = motor ? PathConstraints.PROFILE_MARGIN * kinematics.maxForwardAccel : 0.0;
        this.motorVFree = motor ? kinematics.freeSpeed : 0.0;
        this.faceMode = heading.isFaceTarget();
        double wheelMax = kinematics != null && moduleRadius > 0.0
                ? PathConstraints.PROFILE_MARGIN * kinematics.maxAllowedModuleVelocity : Double.NaN;

        this.length = HeadingProfile.pathLength(segments);
        if (!(length > 0.0)) {
            throw new IllegalArgumentException("SpeedProfile: path length must be > 0");
        }

        // Stops (7.10.1): every STOP entry strictly inside (0, L).
        List<double[]> stops = new ArrayList<>();
        for (Segment g : segments) {
            if (g.type == Segment.Type.STOP && g.sStart > S_EPS && g.sStart < length - S_EPS) {
                stops.add(new double[] {g.sStart, g.dwell});
            }
        }
        stops.sort((x, y) -> Double.compare(x[0], y[0]));
        stopS = new double[stops.size()];
        stopDwell = new double[stops.size()];
        for (int i = 0; i < stops.size(); i++) {
            stopS[i] = stops.get(i)[0];
            stopDwell[i] = stops.get(i)[1];
        }

        // 7.3.1 boundaries.
        List<Double> b = new ArrayList<>();
        b.add(0.0);
        b.add(length);
        for (Segment g : segments) {
            b.add(g.sStart);
        }
        for (double w : waypointS) {
            b.add(w);
        }
        for (double s : stopS) {
            b.add(s);
        }
        for (double s : heading.breakpointsS()) {
            b.add(s);
        }
        if (heading.isFaceTarget()) {
            for (double s = FT_SPAN; s < length; s += FT_SPAN) {
                b.add(s);
            }
            for (double s : heading.freezeBoundaries()) {
                b.add(s);
            }
        }
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
        js = new double[kCount + 1];
        for (int j = 0; j <= kCount; j++) {
            js[j] = merged.get(j);
        }

        sStart = new double[kCount];
        len = new double[kCount];
        kappa = new double[kCount];
        dtheta = new double[kCount];
        double[] dtheta2 = new double[kCount];
        boolean[] frozen = new boolean[kCount];
        int[] segIdx = new int[kCount];
        kind = new Kind[kCount];
        for (int k = 0; k < kCount; k++) {
            sStart[k] = js[k];
            len[k] = js[k + 1] - js[k];
            double mid = 0.5 * (js[k] + js[k + 1]);
            int gi = segmentAt(segments, mid);
            segIdx[k] = gi;
            Segment g = segments.get(gi);
            kappa[k] = g.type == Segment.Type.ARC ? g.sigma / g.r : 0.0;
            kind[k] = g.type == Segment.Type.ARC ? Kind.ARC : Kind.LINE;
            if (heading.isFaceTarget()) {
                frozen[k] = heading.isFrozen(mid);
                if (!frozen[k]) {
                    double[] m = faceMaxima(heading, g, js[k], js[k + 1]);
                    dtheta[k] = m[0];
                    dtheta2[k] = m[1];
                }
            } else {
                dtheta[k] = heading.dTheta(mid);
            }
        }

        // Junction stop flags and withVelocity point caps.
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
        {
            int last = waypointS.length - 1;
            for (int i = 1; i < Math.min(last, pointVelocity.length); i++) {
                double pv = pointVelocity[i];
                if (Double.isNaN(pv) || !(pv > 0.0)) {
                    continue; // none, or 0 (a stop: already a STOP entry)
                }
                int j = junctionNear(waypointS[i]);
                if (j > 0 && j < kCount && !isStop[j]) {
                    wpVel[j] = Math.min(wpVel[j], pv);
                }
            }
        }

        // 7.8 ARC_VAR selection.
        for (int j = 1; j < kCount; j++) {
            if (!Double.isFinite(wpVel[j])) {
                continue;
            }
            if (kind[j - 1] == Kind.LINE || kind[j] == Kind.LINE || segIdx[j - 1] != segIdx[j]) {
                continue;
            }
            int gi = segIdx[j];
            double capVar = Double.POSITIVE_INFINITY;
            for (int k = 0; k < kCount; k++) {
                if (segIdx[k] == gi) {
                    capVar = Math.min(capVar, limits(k, Kind.ARC_VAR, dtheta2[k], wheelMax, moduleRadius)[0]);
                }
            }
            if (wpVel[j] < capVar - 1e-12) {
                for (int k = 0; k < kCount; k++) {
                    if (segIdx[k] == gi) {
                        kind[k] = Kind.ARC_VAR;
                    }
                }
            }
        }

        // 7.4 span limits.
        cap = new double[kCount];
        aA = new double[kCount];
        aD = new double[kCount];
        for (int k = 0; k < kCount; k++) {
            double[] lim = limits(k, kind[k], dtheta2[k], wheelMax, moduleRadius);
            cap[k] = lim[0];
            aA[k] = lim[1];
            aD[k] = lim[2];
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
                double jump;
                if (heading.isFaceTarget()) {
                    jump = frozen[j - 1] != frozen[j] ? Math.abs(heading.faceSlope(js[j])) : 0.0;
                } else {
                    jump = Math.abs(dtheta[j] - dtheta[j - 1]);
                }
                if (jump > 1e-12) {
                    v = Math.min(v, kinkNum / jump);
                }
            }
            if (isStop[j]) {
                v = 0.0;
            }
            jCap[j] = v;
        }

        // Sub-profiles and passes (7.6.2, 7.10.2).
        List<Integer> bounds = new ArrayList<>();
        bounds.add(0);
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
            for (int j = first; j <= last; j++) {
                lb[j] = jCap[j];
            }
            lb[first] = cap[first]; // normal caps at a sub-profile start, not 0 (7.10.2)
            lb[last] = 0.0;
            for (int j = last - 1; j >= first; j--) {
                lb[j] = Math.min(lb[j], Math.sqrt(lb[j + 1] * lb[j + 1] + 2.0 * aD[j] * len[j]));
                if (needsVJerk(j, first)) {
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
    }

    // ---------------------------------------------------------------- build helpers

    private static void requireFinite(String name, double x) {
        if (!Double.isFinite(x)) {
            throw new IllegalArgumentException("SpeedProfile: " + name + " must be finite, got " + x);
        }
    }

    private static int segmentAt(List<Segment> segments, double s) {
        int found = -1;
        for (int i = 0; i < segments.size(); i++) {
            Segment g = segments.get(i);
            if (g.type == Segment.Type.STOP || g.length <= 0.0) {
                continue;
            }
            if (found < 0 || g.sStart <= s) {
                found = i;
            }
        }
        if (found < 0) {
            throw new IllegalArgumentException("SpeedProfile: no positive-length entry");
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

    /** @return {theta'_max, theta''_max} over [s0, s1] on entry g (6.9) */
    private static double[] faceMaxima(HeadingProfile h, Segment g, double s0, double s1) {
        List<Double> pts = new ArrayList<>();
        for (int i = 0; i <= FT_SAMPLES; i++) {
            pts.add(s0 + (s1 - s0) * i / FT_SAMPLES);
        }
        if (g.type == Segment.Type.LINE) {
            // closest approach (theta' extremum) and u = +-h/sqrt(3) from it (theta'' extrema)
            double px = h.faceTarget().getX() - g.a.getX();
            double py = h.faceTarget().getY() - g.a.getY();
            double sc = g.sStart + px * g.t.getX() + py * g.t.getY();
            double dist = Math.abs(px * g.t.getY() - py * g.t.getX());
            for (double x : new double[] {sc, sc - dist / Math.sqrt(3.0), sc + dist / Math.sqrt(3.0)}) {
                if (x > s0 && x < s1) {
                    pts.add(x);
                }
            }
        } else if (g.type == Segment.Type.ARC) {
            // closest and farthest approach: the arc points on the line from C through the target
            double phiT = Math.atan2(h.faceTarget().getY() - g.center.getY(), h.faceTarget().getX() - g.center.getX());
            for (double phi : new double[] {phiT, phiT + Math.PI}) {
                double x = g.sStart + g.r * PathMath.mod2pi(g.sigma * (phi - g.phi0));
                if (x > s0 && x < s1) {
                    pts.add(x);
                }
            }
        }
        double m1 = 0.0;
        double m2 = 0.0;
        double at1 = Double.NaN;
        double at2 = Double.NaN;
        for (double s : pts) {
            if (h.isFrozen(s)) {
                continue;
            }
            double v1 = Math.abs(h.faceSlope(s));
            double v2 = Math.abs(h.faceSlope2(s));
            if (v1 > m1) {
                m1 = v1;
                at1 = s;
            }
            if (v2 > m2) {
                m2 = v2;
                at2 = s;
            }
        }
        // refine around the sampled maxima (an interior extremum between two samples)
        double w = (s1 - s0) / FT_SAMPLES;
        for (int i = -FT_SAMPLES; i <= FT_SAMPLES; i++) {
            double x1 = at1 + w * i / FT_SAMPLES;
            if (!Double.isNaN(at1) && x1 > s0 && x1 < s1 && !h.isFrozen(x1)) {
                m1 = Math.max(m1, Math.abs(h.faceSlope(x1)));
            }
            double x2 = at2 + w * i / FT_SAMPLES;
            if (!Double.isNaN(at2) && x2 > s0 && x2 < s1 && !h.isFrozen(x2)) {
                m2 = Math.max(m2, Math.abs(h.faceSlope2(x2)));
            }
        }
        return new double[] {m1, m2};
    }

    /** spanLimits of 7.17: {cap, aA, aD} for span k as the given kind. */
    private double[] limits(int k, Kind kd, double dth2, double wheelMax, double moduleRadius) {
        double vc = c.vMax();
        double a1 = c.aAcc();
        double a2 = c.aDec();
        double share = Math.sqrt(1.0 - ARC_RHO * ARC_RHO);
        if (kd == Kind.ARC) {
            vc = Math.min(vc, Math.sqrt(c.aLat() / Math.abs(kappa[k])));
            a1 = 0.0;
            a2 = 0.0;
        } else if (kd == Kind.ARC_VAR) {
            vc = Math.min(vc, Math.sqrt(ARC_RHO * c.aLat() / Math.abs(kappa[k])));
            a1 *= share;
            a2 *= share;
        }
        double dth = Math.abs(dtheta[k]);
        double alphaShare = faceMode ? FACE_TARGET_ALPHA_SPLIT : 1.0;
        if (dth > 0.0) {
            vc = Math.min(vc, c.omegaMax() / dth);
            if (a1 > 0.0) {
                a1 = Math.min(a1, alphaShare * c.alphaMax() / dth);
            }
            if (a2 > 0.0) {
                a2 = Math.min(a2, alphaShare * c.alphaMax() / dth);
            }
        }
        if (dth2 > 0.0) {
            vc = Math.min(vc, Math.sqrt(FACE_TARGET_ALPHA_SPLIT * c.alphaMax() / dth2));
        }
        if (!Double.isNaN(wheelMax)) {
            vc = Math.min(vc, wheelMax / (1.0 + dth * moduleRadius));
        }
        if (kd == Kind.ARC_VAR && motor) {
            a1 = Math.min(a1, motorA0 * Math.max(0.0, 1.0 - vc / motorVFree));
        }
        return new double[] {vc, a1, a2};
    }

    private boolean needsVJerk(int j, int first) {
        if (kind[j] == Kind.LINE && j > first && kind[j - 1] == Kind.ARC) {
            return true; // line after a constant-speed arc
        }
        return kind[j] == Kind.ARC_VAR && (j == first || kind[j - 1] != Kind.ARC_VAR);
    }

    /** Room to brake after an arc (7.8): largest v at junction j, a = 0, that meets every later constraint. */
    private double vJerk(int j, int last, double[] lb) {
        double jerk = c.jerkMax();
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
            if (q == last) {
                dist[i] = js[q] - handOver - js[j];
                vt[i] = kP * handOver;
            } else {
                dist[i] = js[q] - js[j];
                vt[i] = lb[q];
            }
        }
        double hi = lb[j];
        if (roomOk(hi, dist, vt, dq, jerk)) {
            return hi;
        }
        double lo = 0.0;
        for (int it = 0; it < ROOM_TO_BRAKE_ITER; it++) {
            double mid = 0.5 * (lo + hi);
            if (roomOk(mid, dist, vt, dq, jerk)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    private static boolean roomOk(double v, double[] dist, double[] vt, double[] dq, double jerk) {
        for (int i = 0; i < dist.length; i++) {
            if (v <= vt[i]) {
                continue;
            }
            if (SpeedLaw.stopDist(v, 0.0, vt[i], jerk, dq[i]) + SpeedLaw.STOP_DIST_MARGIN > dist[i]) {
                return false;
            }
        }
        return true;
    }

    // ---------------------------------------------------------------- speed-up curve (7.6.2 F_k, 7.9)

    private boolean motorOn(int k) {
        return motor && kind[k] == Kind.LINE && aA[k] > 0.0;
    }

    private double knee(int k) {
        return aA[k] < motorA0 ? motorVFree * (1.0 - aA[k] / motorA0) : 0.0;
    }

    /** @return distance to speed up from v1 to v2 on span k [m] */
    double upDist(int k, double v1, double v2) {
        if (v2 <= v1) {
            return 0.0;
        }
        if (!motorOn(k)) {
            return (v2 * v2 - v1 * v1) / (2.0 * aA[k]);
        }
        return motorDist(aA[k], motorA0, motorVFree, v1, v2);
    }

    /** @return time to speed up from v1 to v2 on span k [s] */
    double upTime(int k, double v1, double v2) {
        if (v2 <= v1) {
            return 0.0;
        }
        if (!motorOn(k)) {
            return (v2 - v1) / aA[k];
        }
        return motorTime(aA[k], motorA0, motorVFree, v1, v2);
    }

    /** @return speed after distance d from v1 on span k (F_k, 7.6.2) */
    double upSpeed(int k, double v1, double d) {
        if (aA[k] <= 0.0 || d <= 0.0) {
            return v1;
        }
        if (!motorOn(k)) {
            return Math.sqrt(v1 * v1 + 2.0 * aA[k] * d);
        }
        return motorSpeed(aA[k], motorA0, motorVFree, v1, d);
    }

    /** Distance from v1 to v2 under {@code a(v) = min(aT, a0 (1 - v/vFree))} (7.9). */
    static double motorDist(double aT, double a0, double vFree, double v1, double v2) {
        if (v2 <= v1) {
            return 0.0;
        }
        if (v2 >= vFree) {
            return Double.POSITIVE_INFINITY;
        }
        double vc = aT < a0 ? vFree * (1.0 - aT / a0) : 0.0;
        double d = 0.0;
        double lo = v1;
        if (lo < vc) {
            double vm = Math.min(v2, vc);
            d += (vm * vm - lo * lo) / (2.0 * aT);
            lo = vm;
        }
        if (v2 > lo) {
            d += (vFree / a0) * (vFree * Math.log((vFree - lo) / (vFree - v2)) - (v2 - lo));
        }
        return d;
    }

    /** Time from v1 to v2 under {@code a(v) = min(aT, a0 (1 - v/vFree))} (7.9). */
    static double motorTime(double aT, double a0, double vFree, double v1, double v2) {
        if (v2 <= v1) {
            return 0.0;
        }
        if (v2 >= vFree) {
            return Double.POSITIVE_INFINITY;
        }
        double vc = aT < a0 ? vFree * (1.0 - aT / a0) : 0.0;
        double t = 0.0;
        double lo = v1;
        if (lo < vc) {
            double vm = Math.min(v2, vc);
            t += (vm - lo) / aT;
            lo = vm;
        }
        if (v2 > lo) {
            t += (vFree / a0) * Math.log((vFree - lo) / (vFree - v2));
        }
        return t;
    }

    /** Speed after distance d from v1 under the motor curve: bisection on motorDist (7.9). */
    static double motorSpeed(double aT, double a0, double vFree, double v1, double d) {
        if (v1 >= vFree) {
            return v1;
        }
        double lo = v1;
        double hi = vFree * (1.0 - 1e-12);
        if (motorDist(aT, a0, vFree, v1, hi) <= d) {
            return hi;
        }
        for (int i = 0; i < MOTOR_CURVE_ITER; i++) {
            double mid = 0.5 * (lo + hi);
            if (motorDist(aT, a0, vFree, v1, mid) < d) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return 0.5 * (lo + hi);
    }

    /** Closed-form span time (7.7, 7.9). */
    private double spanTimeOf(int k, double vin, double vout) {
        double l = len[k];
        if (aA[k] <= 0.0 && aD[k] <= 0.0) {
            if (!(vin > 0.0)) {
                throw new IllegalStateException("SpeedProfile: constant-speed span " + k + " entered at 0 (bug)");
            }
            return l / vin;
        }
        double vc = cap[k];
        double brake = aD[k];
        // Cruise if the speed-up to vc and the slow-down from vc fit.
        double q1 = upDist(k, vin, vc);
        double q2 = l - (vc * vc - vout * vout) / (2.0 * brake);
        if (q1 <= q2) {
            return upTime(k, vin, vc) + (q2 - q1) / vc + (vc - vout) / brake;
        }
        double vp;
        if (!motorOn(k)) {
            double qs = (vout * vout - vin * vin + 2.0 * brake * l) / (2.0 * (aA[k] + brake));
            vp = Math.sqrt(Math.max(0.0, vin * vin + 2.0 * aA[k] * qs));
        } else {
            double lo = Math.max(vin, vout);
            double hi = vc;
            for (int i = 0; i < PEAK_ITER; i++) {
                double mid = 0.5 * (lo + hi);
                if (upDist(k, vin, mid) + (mid * mid - vout * vout) / (2.0 * brake) < l) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            vp = 0.5 * (lo + hi);
        }
        vp = Math.max(vp, Math.max(vin, vout));
        return upTime(k, vin, vp) + (vp - vout) / brake;
    }

    // ---------------------------------------------------------------- queries

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
     *     junction it reads the later span: at a stop s_c that is the restart envelope of the next
     *     sub-profile. A robot approaching a stop uses {@link #vBack(double, double)}.
     */
    double vBack(double s) {
        return vBackIn(spanAt(s), s);
    }

    /**
     * vBack clamped to the sub-profile that ends at sGoal (7.13.10 step 1): s is clamped into that
     * sub-profile, so at s = s_c the approaching robot reads 0, not the restart envelope.
     *
     * @param s     progress
     * @param sGoal end of the current sub-profile (a stop or L), as from {@link #nextStopS(double)}
     * @return vBack at s within that sub-profile
     */
    double vBack(double s, double sGoal) {
        int k = spanInSub(s, sGoal);
        return vBackIn(k, s);
    }

    /** @return full profile speed at s, for logging (7.13.10 step 4); later span at a junction, like vBack(s) */
    double vProf(double s) {
        return vProfIn(spanAt(s), s);
    }

    /**
     * vProf clamped to the sub-profile that ends at sGoal (7.13.10).
     *
     * @param s     progress
     * @param sGoal end of the current sub-profile (a stop or L)
     * @return vProf at s within that sub-profile
     */
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
        int p = subProfileEndingAt(sGoal);
        int first = subFirst[p];
        int last = subLast[p] - 1;
        return Math.max(first, Math.min(last, spanAt(s)));
    }

    private int subProfileEndingAt(double sGoal) {
        int best = subFirst.length - 1;
        double bestD = Double.POSITIVE_INFINITY;
        for (int p = 0; p < subLast.length; p++) {
            double d = Math.abs(js[subLast[p]] - sGoal);
            if (d < bestD) {
                bestD = d;
                best = p;
            }
        }
        return best;
    }

    /** @return speed cap vCap of the span at s (7.4) */
    double capAt(double s) {
        return cap[spanAt(s)];
    }

    /** @return braking limit aD of the span at s (7.13.9: its own aD on a line or ARC_VAR, aDec on a constant arc) */
    double decelAt(double s) {
        int k = spanAt(s);
        return kind[k] == Kind.ARC ? c.aDec() : aD[k];
    }

    /** @return Amax(v, s): acceleration limit at speed v in the span at s, with the motor curve (7.9, 7.13.9) */
    double accelLimit(double v, double s) {
        int k = spanAt(s);
        if (kind[k] == Kind.ARC) {
            return 0.0;
        }
        double a = aA[k];
        if (motor) {
            a = Math.min(a, motorA0 * Math.max(0.0, 1.0 - v / motorVFree));
        }
        return a;
    }

    /** @return true if s is on a constant-speed arc, where the law has no "up" candidate (7.13.9) */
    boolean onConstantArc(double s) {
        return kind[spanAt(s)] == Kind.ARC;
    }

    /** @return junctions with s in (sFrom, sGoal), ascending, for the law's constraint list (7.13.9) */
    List<Junction> junctionsAhead(double sFrom, double sGoal) {
        int n = len.length + 1;
        double[] ss = new double[n];
        double[] vt = new double[n];
        double[] d = new double[n];
        boolean[] e = new boolean[n];
        int m = fillJunctionsAhead(sFrom, sGoal, ss, vt, d, e);
        List<Junction> out = new ArrayList<>(m);
        for (int i = 0; i < m; i++) {
            out.add(new Junction(ss[i], vt[i], d[i], e[i]));
        }
        return out;
    }

    /**
     * Allocation-free form of {@link #junctionsAhead(double, double)} for the 50 Hz law: writes the
     * junctions into the arrays (each at least spanCount() + 1 long).
     *
     * @return the number of junctions written
     */
    int fillJunctionsAhead(double sFrom, double sGoal, double[] outS, double[] outVt, double[] outD,
                           boolean[] outEntry) {
        int n = 0;
        int k = spanAt(sFrom);
        double dmin = Double.POSITIVE_INFINITY;
        for (int j = k + 1; j < js.length; j++) {
            if (kind[j - 1] != Kind.ARC) {
                dmin = Math.min(dmin, aD[j - 1]);
            }
            if (js[j] >= sGoal) {
                break;
            }
            if (js[j] <= sFrom) {
                continue;
            }
            outS[n] = js[j];
            outVt[n] = vB[j];
            outD[n] = Double.isFinite(dmin) ? dmin : c.aDec();
            outEntry[n] = j < len.length && kind[j - 1] == Kind.LINE && kind[j] != Kind.LINE;
            n++;
        }
        return n;
    }

    /**
     * D for a constraint at sTo seen from sFrom (7.13.4): the smallest aD of the line and ARC_VAR spans
     * overlapping [sFrom, sTo]; aDec if there is none.
     */
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

    /** @return the planned time including dwells, from the offline jerk law (7.12, 7.13.6, 7.10.5) */
    double profileTime() {
        if (Double.isNaN(profileTimeCache)) {
            double t = startDwell; // stop(d) on P_0 waits before the start (8.6)
            boolean hit = false;
            for (int p = 0; p < subFirst.length; p++) {
                double s0 = js[subFirst[p]];
                double s1 = js[subLast[p]];
                SpeedLaw.OfflineResult r = SpeedLaw.offlineRun(this, p == 0 ? v0 : 0.0, s0, s1, OFFLINE_DT);
                t += r.time() + dwellAt(s1);
                hit |= r.guardHit();
            }
            profileGuardHit = hit;
            profileTimeCache = t;
        }
        return profileTimeCache;
    }

    /**
     * @return true if the offline run of {@link #profileTime()} hit its guard in any sub-profile (a bug:
     *     "profile simulation did not reach the goal", reported on warning C2's channel, 7.13.6)
     */
    boolean profileGuardHit() {
        profileTime();
        return profileGuardHit;
    }

    // ---------------------------------------------------------------- extra accessors (logging, tests)

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

    /** @return position tolerance of the hand-over tail [m] */
    double posTol() {
        return posTol;
    }

    /** @return hand-over distance d_h used by the profile (8.3.7) */
    double handOver() {
        return handOver;
    }

    /** @return kind of span k */
    Kind spanKind(int k) {
        return kind[k];
    }

    /** @return cap of span k */
    double spanCap(int k) {
        return cap[k];
    }

    /** @return speed-up limit at rest of span k */
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

    /** @return theta' of span k (theta'_max in face-target mode) */
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
