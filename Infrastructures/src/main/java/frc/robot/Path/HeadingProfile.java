package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Reference heading theta(s) and theta'(s): piecewise linear through the anchors, or the
 * face-target heading (design doc section 6). Built per command start from the (flipped) path.
 */
final class HeadingProfile {

    /** Two anchors closer than this along the path are one anchor; the later one wins (6.2, W3) [m]. */
    private static final double ANCHOR_MERGE = 1e-9;
    /** Sampling step used to find the face-target freeze intervals [m] (6.9). */
    private static final double FREEZE_SCAN_STEP = 0.01;
    /** Bisection iterations for a freeze boundary. */
    private static final int FREEZE_BISECT_ITER = 60;

    // Anchor mode: the knots of the piecewise-linear theta(s), theta stored unwrapped (6.3).
    private final double[] knotS;
    private final double[] knotTheta;

    // Face-target mode (6.9).
    private final Translation2d target;
    private final List<Segment> segments;
    private final double length;
    private final double[] freezeStart;
    private final double[] freezeEnd;
    private final double[] freezeTheta;

    private HeadingProfile(double[] knotS, double[] knotTheta, Translation2d target, List<Segment> segments,
                           double length, double[] freezeStart, double[] freezeEnd, double[] freezeTheta) {
        this.knotS = knotS;
        this.knotTheta = knotTheta;
        this.target = target;
        this.segments = segments;
        this.length = length;
        this.freezeStart = freezeStart;
        this.freezeEnd = freezeEnd;
        this.freezeTheta = freezeTheta;
    }

    /**
     * Builds the profile (6.13).
     *
     * @param path   the active (already flipped) path; uses table(), points(), sAtPoint, faceTarget()
     * @param theta0 measured start heading [rad], used when P_0 has no angle (6.8)
     * @return the profile
     */
    static HeadingProfile build(Path path, double theta0) {
        requireFiniteTheta0(theta0);
        Optional<Translation2d> face = path.faceTarget();
        if (face.isPresent()) {
            return faceTarget(face.get(), segmentsOf(path.table()));
        }
        List<Point> points = path.points();
        int n = points.size();
        double[] s = new double[n];
        double[] angle = new double[n];
        for (int i = 0; i < n; i++) {
            s[i] = path.sAtPoint(i);
            angle[i] = points.get(i).angle().map(Rotation2d::getRadians).orElse(Double.NaN);
        }
        return anchors(s, angle, theta0, path.length());
    }

    /**
     * Anchor-mode profile from plain numbers (6.1 to 6.3, 6.8).
     *
     * @param pointS     s of each point the user wrote, by user index (merged points share one s)
     * @param pointAngle angle of each point [rad], NaN for a point without {@code withAngle}
     * @param theta0     measured start heading [rad], used when no anchor sits at s = 0
     * @param length     path length L [m]
     * @return the profile
     */
    static HeadingProfile anchors(double[] pointS, double[] pointAngle, double theta0, double length) {
        requireFiniteTheta0(theta0);
        if (pointS.length != pointAngle.length) {
            throw new IllegalArgumentException("HeadingProfile: pointS and pointAngle differ in length");
        }
        List<double[]> anchors = new ArrayList<>();
        for (int i = 0; i < pointS.length; i++) {
            if (!Double.isNaN(pointAngle[i])) {
                anchors.add(new double[] {pointS[i], pointAngle[i]});
            }
        }
        // Stable sort by s keeps the user order at equal s, so the later point is last.
        anchors.sort((x, y) -> Double.compare(x[0], y[0]));
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
        ks[0] = 0.0;
        kt[0] = thetaStart;
        for (int i = first, j = 1; i < merged.size(); i++, j++) {
            double prev = kt[j - 1];
            ks[j] = merged.get(i)[0];
            kt[j] = prev + PathMath.shortTurn(merged.get(i)[1] - prev);
        }
        return new HeadingProfile(ks, kt, null, null, length, null, null, null);
    }

    /**
     * Face-target profile (6.9): theta_FT(s) = atan2(target - x(s)), frozen inside {@link Tracker#R_FACE_MIN}.
     *
     * @param target   face target in the active frame
     * @param segments geometry entries in order (LINE / ARC / STOP)
     * @return the profile
     */
    static HeadingProfile faceTarget(Translation2d target, List<Segment> segments) {
        double len = pathLength(segments);
        List<double[]> intervals = new ArrayList<>();
        int n = Math.max(1, (int) Math.ceil(len / FREEZE_SCAN_STEP));
        double prevS = 0.0;
        boolean prevIn = inside(target, segments, 0.0);
        double start = prevIn ? 0.0 : Double.NaN;
        for (int i = 1; i <= n; i++) {
            double s = Math.min(len, len * i / n);
            boolean in = inside(target, segments, s);
            if (in != prevIn) {
                double lo = prevS;
                double hi = s;
                for (int it = 0; it < FREEZE_BISECT_ITER; it++) {
                    double mid = 0.5 * (lo + hi);
                    if (inside(target, segments, mid) == prevIn) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                double b = 0.5 * (lo + hi);
                if (in) {
                    start = b;
                } else {
                    intervals.add(new double[] {start, b});
                }
            }
            prevS = s;
            prevIn = in;
        }
        if (prevIn) {
            intervals.add(new double[] {start, len});
        }
        double[] fs = new double[intervals.size()];
        double[] fe = new double[intervals.size()];
        double[] ft = new double[intervals.size()];
        for (int i = 0; i < intervals.size(); i++) {
            fs[i] = intervals.get(i)[0];
            fe[i] = intervals.get(i)[1];
            ft[i] = rawTheta(target, segments, fs[i]);
        }
        return new HeadingProfile(null, null, target, List.copyOf(segments), len, fs, fe, ft);
    }

    /** @return reference heading at s [rad], unwrapped (6.3); wrap it before comparing */
    double theta(double s) {
        if (target != null) {
            int f = freezeIndex(s);
            return f >= 0 ? freezeTheta[f] : rawTheta(target, segments, s);
        }
        if (s <= knotS[0]) {
            return knotTheta[0];
        }
        int last = knotS.length - 1;
        if (s >= knotS[last]) {
            return knotTheta[last];
        }
        int i = piece(s);
        double f = (s - knotS[i]) / (knotS[i + 1] - knotS[i]);
        return knotTheta[i] + f * (knotTheta[i + 1] - knotTheta[i]);
    }

    /** @return theta'(s) [rad/m]; 0 after the last anchor (6.4) */
    double dTheta(double s) {
        if (target != null) {
            return freezeIndex(s) >= 0 ? 0.0 : faceSlope(s);
        }
        int last = knotS.length - 1;
        if (last == 0 || s >= knotS[last]) {
            return 0.0;
        }
        int i = s <= knotS[0] ? 0 : piece(s);
        return (knotTheta[i + 1] - knotTheta[i]) / (knotS[i + 1] - knotS[i]);
    }

    /** @return s of the piecewise-linear knots (anchors); empty array for a face target. Used for kink caps (7.4.6) */
    double[] breakpointsS() {
        return target != null ? new double[0] : knotS.clone();
    }

    /** @return true if this is the face-target heading (6.9) */
    boolean isFaceTarget() {
        return target != null;
    }

    /** @return the face target in the active frame, or null if none */
    Translation2d faceTarget() {
        return target;
    }

    /** @return heading at the end of the path, theta(L) (8.7.2) */
    double endHeading() {
        if (target != null) {
            return theta(length);
        }
        return knotTheta[knotTheta.length - 1];
    }

    // ---------------------------------------------------------------- face-target helpers (6.9)

    /** @return true if s lies in a freeze interval (abs(target - x(s)) &lt; Tracker.R_FACE_MIN); closed at L (6.9) */
    boolean isFrozen(double s) {
        return target != null && freezeIndex(s) >= 0;
    }

    /** @return the freeze boundaries (entry and exit s of every freeze interval), ascending; empty for anchors */
    double[] freezeBoundaries() {
        if (target == null) {
            return new double[0];
        }
        double[] out = new double[2 * freezeStart.length];
        for (int i = 0; i < freezeStart.length; i++) {
            out[2 * i] = freezeStart[i];
            out[2 * i + 1] = freezeEnd[i];
        }
        return out;
    }

    /** @return theta'_FT(s) without the freeze, {@code cross(T, pT)/abs(pT)^2} with pT = target - x(s) [rad/m] */
    double faceSlope(double s) {
        double[] g = evaluate(segments, s);
        double px = target.getX() - g[0];
        double py = target.getY() - g[1];
        double d2 = px * px + py * py;
        if (d2 < 1e-18) {
            return 0.0;
        }
        return (py * g[2] - px * g[3]) / d2;
    }

    /** @return theta''_FT(s) without the freeze [rad/m^2], analytic (line and arc) */
    double faceSlope2(double s) {
        double[] g = evaluate(segments, s);
        double px = target.getX() - g[0];
        double py = target.getY() - g[1];
        double d2 = px * px + py * py;
        if (d2 < 1e-18) {
            return 0.0;
        }
        double n = py * g[2] - px * g[3];
        double pDotT = px * g[2] + py * g[3];
        double kappa = g[4];
        return (-kappa * pDotT * d2 + 2.0 * n * pDotT) / (d2 * d2);
    }

    private static void requireFiniteTheta0(double theta0) {
        if (!Double.isFinite(theta0)) {
            throw new IllegalArgumentException("HeadingProfile: measured start heading theta0 must be finite, got "
                    + theta0);
        }
    }

    private int freezeIndex(double s) {
        for (int i = 0; i < freezeStart.length; i++) {
            boolean closedAtEnd = freezeEnd[i] >= length && s >= length; // interval ending at L includes L
            if (s > freezeStart[i] && (s < freezeEnd[i] || closedAtEnd)) {
                return i;
            }
            if (freezeStart[i] == 0.0 && s <= 0.0 && freezeEnd[i] > 0.0) {
                return i;
            }
        }
        return -1;
    }

    private int piece(double s) {
        int i = Arrays.binarySearch(knotS, s);
        if (i < 0) {
            i = -i - 2;
        }
        return Math.max(0, Math.min(knotS.length - 2, i));
    }

    private static boolean inside(Translation2d target, List<Segment> segments, double s) {
        double[] g = evaluate(segments, s);
        return Math.hypot(target.getX() - g[0], target.getY() - g[1]) < Tracker.R_FACE_MIN;
    }

    private static double rawTheta(Translation2d target, List<Segment> segments, double s) {
        double[] g = evaluate(segments, s);
        double px = target.getX() - g[0];
        double py = target.getY() - g[1];
        if (px == 0.0 && py == 0.0) {
            return 0.0;
        }
        return Math.atan2(py, px);
    }

    // ---------------------------------------------------------------- plain geometry from Segment data

    /** @return the entries of the table, in order */
    static List<Segment> segmentsOf(SegmentTable table) {
        List<Segment> out = new ArrayList<>(table.size());
        for (int k = 0; k < table.size(); k++) {
            out.add(table.segment(k));
        }
        return out;
    }

    /** @return total length: sEnd of the last entry */
    static double pathLength(List<Segment> segments) {
        double len = 0.0;
        for (Segment g : segments) {
            len = Math.max(len, g.sEnd());
        }
        return len;
    }

    /**
     * Evaluates the geometry at s from the Segment data (5.9.2), clamped to [0, L]; at a boundary the
     * later positive-length entry is used. Used only for the face-target heading and its caps.
     *
     * @return {x, y, Tx, Ty, kappa}
     */
    static double[] evaluate(List<Segment> segments, double s) {
        Segment seg = null;
        for (Segment g : segments) {
            if (g.type == Segment.Type.STOP || g.length <= 0.0) {
                continue;
            }
            if (seg == null || g.sStart <= s) {
                seg = g;
            }
        }
        if (seg == null) {
            throw new IllegalStateException("HeadingProfile: path has no positive-length entry");
        }
        double q = Math.max(0.0, Math.min(seg.length, s - seg.sStart));
        if (seg.type == Segment.Type.LINE) {
            return new double[] {seg.a.getX() + seg.t.getX() * q, seg.a.getY() + seg.t.getY() * q,
                seg.t.getX(), seg.t.getY(), 0.0};
        }
        double phi = seg.phi0 + seg.sigma * q / seg.r;
        double c = Math.cos(phi);
        double sn = Math.sin(phi);
        return new double[] {seg.center.getX() + seg.r * c, seg.center.getY() + seg.r * sn,
            -seg.sigma * sn, seg.sigma * c, seg.sigma / seg.r};
    }
}
