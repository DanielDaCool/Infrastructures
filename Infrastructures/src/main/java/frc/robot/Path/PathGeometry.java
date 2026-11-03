package frc.robot.Path;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.StringJoiner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Path.PathFollowerConfig.FlipMode;

/**
 * The built geometry of a path: LINE / ARC / STOP entries in order, with arc-length evaluation,
 * windowed projection and the alliance flip (design doc sections 5.3 to 5.15). {@link #build}
 * turns the user's points into the table: merging, PASS markers, corner circles, radius rule and
 * shrink loop, tangents, sweeps. Immutable after construction.
 */
final class PathGeometry {

    /** Smallest arc radius [m] (5.7.3). */
    static final double R_MIN = 0.05;
    /** Largest automatic radius [m] (5.7.2). */
    static final double R_MAX = 2.0;
    /** Bulge distance [m] (5.7.2). */
    static final double D_BULGE = 0.5;
    /** Largest allowed line deviation, 30 deg [rad] (5.7.1, F2 and F3). */
    static final double EPS_DEV = Math.toRadians(30.0);
    /** Turn above which a corner becomes a stop, 150 deg [rad] (5.6.4). */
    static final double TAU_STOP = Math.toRadians(150.0);
    /** Radius shrink factor per pass (5.7.3). */
    static final double SHRINK = 0.8;
    /** Maximum shrink passes (5.7.3). */
    static final int MAX_ITER = 1000;
    /** Largest miss of a straight point without warning W5 [m] (5.8.3). */
    static final double PASS_TOL = 0.01;
    /** Merge distance [m] (5.8.2). */
    static final double EPS_DUP = 1e-3;
    /** Straight-point angle threshold [rad] (5.3, 5.8.3). */
    static final double EPS_ANG = 1e-4;
    /** Tangent existence, zero-length segments [m] (5.4). */
    static final double EPS_LEN = 1e-6;
    /** Sweep wrap guard [rad] (5.5). */
    static final double EPS_WRAP = 1e-9;
    /** Radius divisor k for an end leg or user-stop leg (5.7.2). */
    static final double RADIUS_DIVISOR_END = 2.0;
    /** Radius divisor k for a shared leg (5.7.2). */
    static final double RADIUS_DIVISOR_SHARED = 4.0;
    /** Robot at an arc center: every arc point is equally near (section 10, EPS_CENTER). */
    static final double EPS_CENTER = 1e-9;
    /** Projection tie distance (section 10, 5.10.4). */
    static final double PROJECTION_TIE = 1e-12;
    /** A change within this of pi turns CCW (section 10, half-turn tie). */
    static final double HALF_TURN_TIE = 1e-9;

    /**
     * One entry of the table (5.9.1). Only the fields of its type are meaningful.
     *
     * @param type        LINE, ARC or STOP
     * @param sStart      s at the start of this entry [m]
     * @param length      length [m]; 0 for STOP
     * @param a           LINE start point, or the STOP point
     * @param t           unit direction T (LINE), or T_out (STOP)
     * @param center      ARC center C
     * @param r           ARC radius
     * @param phi0        ARC start angle on the circle [rad]
     * @param dphi        ARC signed sweep [rad]
     * @param sWp         s of the waypoint on this arc (STOP: sStart)
     * @param tIn         STOP direction of arrival T_in
     * @param cornerIndex STOP user index (for merged points, the kept one); -1 otherwise
     * @param userStop    true if the user wrote stop(); false for an automatic stop corner
     * @param dwell       STOP dwell [s]
     */
    record Segment(Type type, double sStart, double length, Translation2d a, Translation2d t, Translation2d center,
                   double r, double phi0, double dphi, double sWp, Translation2d tIn, int cornerIndex,
                   boolean userStop, double dwell) {

        /** Entry type. */
        enum Type { LINE, ARC, STOP }

        /** @return a LINE from a with unit direction t */
        static Segment line(double sStart, double length, Translation2d a, Translation2d t) {
            return new Segment(Type.LINE, sStart, length, a, t, null, 0, 0, 0, 0, null, -1, false, 0);
        }

        /** @return an ARC about center with radius r, start angle phi0 and signed sweep dphi */
        static Segment arc(double sStart, Translation2d center, double r, double phi0, double dphi, double sWp) {
            return new Segment(Type.ARC, sStart, r * Math.abs(dphi), null, null, center, r, phi0, dphi, sWp,
                    null, -1, false, 0);
        }

        /** @return a zero-length STOP at point p arriving along tIn and leaving along tOut */
        static Segment stop(double sStart, Translation2d p, Translation2d tIn, Translation2d tOut,
                            int cornerIndex, boolean userStop, double dwell) {
            return new Segment(Type.STOP, sStart, 0, p, tOut, null, 0, 0, 0, sStart, tIn, cornerIndex,
                    userStop, dwell);
        }

        /** @return sign(dphi): +1 left, -1 right */
        double sigma() {
            return Math.signum(dphi);
        }

        /** @return s at the end of this entry */
        double sEnd() {
            return sStart + length;
        }
    }

    /**
     * Read-only geometry sample at arc length s (9.11).
     *
     * @param s         arc length [m]
     * @param position  point on the path
     * @param tangent   direction of travel
     * @param curvature signed curvature [1/m], positive for a left turn
     */
    record PathSample(double s, Translation2d position, Rotation2d tangent, double curvature) {
    }

    /**
     * Result of a projection.
     *
     * @param s  s* [m]
     * @param eN signed cross-track error, + = left of the path [m]
     */
    record Projection(double s, double eN) {
    }

    private final List<Segment> segments;
    private final double[] sStarts;
    private final double[] waypointS;
    private final double length;

    /**
     * @param segments  entries in order, line, corner, line, ... (section 5.9.1)
     * @param waypointS s of each point the user wrote, by user index (merged points map to the kept one)
     */
    PathGeometry(List<Segment> segments, double[] waypointS) {
        this.segments = List.copyOf(segments);
        this.waypointS = waypointS.clone();
        this.sStarts = new double[segments.size()];
        for (int k = 0; k < segments.size(); k++) {
            sStarts[k] = segments.get(k).sStart();
        }
        this.length = segments.get(segments.size() - 1).sEnd();
    }

    // ================================================================ queries (5.9)

    /** @return total length L [m] */
    double length() {
        return length;
    }

    /** @return number of entries m */
    int size() {
        return segments.size();
    }

    /** @return entry k */
    Segment segment(int k) {
        return segments.get(k);
    }

    /** @return the entries in order */
    List<Segment> segments() {
        return segments;
    }

    /** @return s of the point the user wrote at index i (5.9.4) */
    double waypointS(int i) {
        return waypointS[i];
    }

    /** @return index of the entry that contains s, later entry at a boundary, never zero-length (5.9.3) */
    int segmentIndexAt(double s) {
        int k = largestStartAtOrBefore(clampS(s));
        while (k > 0 && segments.get(k).length() == 0) {
            k--;
        }
        return k;
    }

    /** @return the sample at s clamped to [0, L]; T is the right limit at a STOP (5.9.2, 5.9.3) */
    PathSample evaluate(double s) {
        double sc = clampS(s);
        return evaluateOn(segmentIndexAt(sc), sc);
    }

    /** @return the sample at s using the largest positive-length entry with sStart &lt; s: T of the leg arrived on */
    PathSample evaluateLeft(double s) {
        double sc = clampS(s);
        int k = segments.size() - 1;
        while (k > 0 && (segments.get(k).length() == 0 || sStarts[k] >= sc)) {
            k--;
        }
        return evaluateOn(k, sc);
    }

    /** @return the sample of entry k at s (s clamped to the entry) */
    private PathSample evaluateOn(int k, double s) {
        Segment seg = segments.get(k);
        double q = Math.max(0.0, Math.min(seg.length(), s - seg.sStart()));
        if (seg.type() == Segment.Type.ARC) {
            double phi = seg.phi0() + seg.sigma() * q / seg.r();
            Translation2d x = new Translation2d(seg.center().getX() + seg.r() * Math.cos(phi),
                    seg.center().getY() + seg.r() * Math.sin(phi));
            return new PathSample(s, x, new Rotation2d(-seg.sigma() * Math.sin(phi), seg.sigma() * Math.cos(phi)),
                    seg.sigma() / seg.r());
        }
        Translation2d x = seg.type() == Segment.Type.LINE
                ? new Translation2d(seg.a().getX() + q * seg.t().getX(), seg.a().getY() + q * seg.t().getY())
                : seg.a();
        return new PathSample(s, x, new Rotation2d(seg.t().getX(), seg.t().getY()), 0.0);
    }

    /** @return s clamped to [0, L]; NaN becomes 0 */
    private double clampS(double s) {
        return Double.isNaN(s) ? 0.0 : Math.max(0.0, Math.min(length, s));
    }

    /** @return the largest index with sStart &lt;= s (binary search; 0 if none) */
    private int largestStartAtOrBefore(double s) {
        int i = Arrays.binarySearch(sStarts, s);
        if (i >= 0) {
            while (i + 1 < sStarts.length && sStarts[i + 1] <= s) {
                i++;
            }
            return i;
        }
        return Math.max(0, -i - 2);
    }

    // ================================================================ projection (5.10, 8.2.3)

    /**
     * Nearest path point with s in [sMin, sMax]; ties keep the s closest to sPrev (5.10.4).
     * Never throws (run time): a NaN sPrev is read as 0, and a NaN window edge is replaced by
     * sPrev, so a NaN window degenerates to the single point s = sPrev.
     *
     * @param p     robot position
     * @param sMin  window start
     * @param sMax  window end
     * @param sPrev previous s*
     * @return s* and e_n
     */
    Projection project(Translation2d p, double sMin, double sMax, double sPrev) {
        if (Double.isNaN(sPrev)) {
            sPrev = 0.0;
        }
        double hi = clampS(Double.isNaN(sMax) ? sPrev : sMax);
        double lo = Math.min(hi, clampS(Double.isNaN(sMin) ? sPrev : sMin));
        double bestS = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int k = segmentIndexAt(lo); k < segments.size(); k++) {
            Segment seg = segments.get(k);
            if (seg.sStart() > hi) {
                break;
            }
            if (seg.sEnd() < lo) {
                continue;
            }
            double sk = Math.max(lo, Math.min(hi, projectOnSegment(k, p, sPrev)));
            double dist = p.getDistance(evaluate(sk).position());
            if (Double.isNaN(bestS) || dist < bestDist - PROJECTION_TIE) {
                bestS = sk;
                bestDist = dist;
            } else if (Math.abs(dist - bestDist) <= PROJECTION_TIE && Math.abs(sk - sPrev) < Math.abs(bestS - sPrev)) {
                bestS = sk;
                bestDist = Math.min(dist, bestDist);
            }
        }
        if (Double.isNaN(bestS)) {
            bestS = lo;
        }
        PathSample sample = evaluate(bestS);
        double eN = -(p.getX() - sample.position().getX()) * sample.tangent().getSin()
                + (p.getY() - sample.position().getY()) * sample.tangent().getCos();
        return new Projection(bestS, eN);
    }

    /**
     * Projection onto one entry, unclamped to the window (5.10.1 to 5.10.3).
     *
     * @param k     entry index
     * @param p     robot position
     * @param sPrev previous s* (used when p is at an arc center)
     * @return s on entry k
     */
    double projectOnSegment(int k, Translation2d p, double sPrev) {
        Segment seg = segments.get(k);
        double q;
        if (seg.type() == Segment.Type.LINE) {
            q = (p.getX() - seg.a().getX()) * seg.t().getX() + (p.getY() - seg.a().getY()) * seg.t().getY();
        } else if (seg.type() == Segment.Type.ARC) {
            double vx = p.getX() - seg.center().getX();
            double vy = p.getY() - seg.center().getY();
            if (Math.hypot(vx, vy) < EPS_CENTER) {
                q = sPrev - seg.sStart();
            } else {
                double sweep = Math.abs(seg.dphi());
                double beta = mod2pi(seg.sigma() * (Math.atan2(vy, vx) - seg.phi0()));
                q = beta <= sweep ? seg.r() * beta : (beta - sweep < 2.0 * Math.PI - beta ? seg.length() : 0.0);
            }
        } else {
            return seg.sStart();
        }
        return seg.sStart() + Math.max(0.0, Math.min(seg.length(), q));
    }

    // ================================================================ flip (5.13)

    /**
     * s values, L and waypoint s do not change; MIRROR flips the sign of sigma, kappa and dphi.
     *
     * @return the flipped table (this for NONE)
     */
    PathGeometry flipped(FlipMode mode, double fieldLength, double fieldWidth) {
        if (mode == FlipMode.NONE) {
            return this;
        }
        List<Segment> out = new ArrayList<>(segments.size());
        for (Segment seg : segments) {
            Translation2d a = seg.a() == null ? null : flip(seg.a(), mode, fieldLength, fieldWidth);
            switch (seg.type()) {
                case LINE -> out.add(Segment.line(seg.sStart(), seg.length(), a, flipDirection(seg.t(), mode)));
                case ARC -> out.add(Segment.arc(seg.sStart(), flip(seg.center(), mode, fieldLength, fieldWidth),
                        seg.r(), wrap(mode == FlipMode.MIRROR ? Math.PI - seg.phi0() : seg.phi0() + Math.PI),
                        mode == FlipMode.MIRROR ? -seg.dphi() : seg.dphi(), seg.sWp()));
                default -> out.add(Segment.stop(seg.sStart(), a, flipDirection(seg.tIn(), mode),
                        flipDirection(seg.t(), mode), seg.cornerIndex(), seg.userStop(), seg.dwell()));
            }
        }
        return new PathGeometry(out, waypointS);
    }

    /** @return a flipped position */
    static Translation2d flip(Translation2d t, FlipMode mode, double fieldLength, double fieldWidth) {
        return switch (mode) {
            case MIRROR -> new Translation2d(fieldLength - t.getX(), t.getY());
            case ROTATE -> new Translation2d(fieldLength - t.getX(), fieldWidth - t.getY());
            default -> t;
        };
    }

    /** @return a flipped heading or direction angle */
    static Rotation2d flip(Rotation2d r, FlipMode mode) {
        return switch (mode) {
            case MIRROR -> new Rotation2d(-r.getCos(), r.getSin());
            case ROTATE -> new Rotation2d(-r.getCos(), -r.getSin());
            default -> r;
        };
    }

    /** @return a flipped unit direction vector (MIRROR: (-x, y); ROTATE: -T) */
    private static Translation2d flipDirection(Translation2d t, FlipMode mode) {
        return switch (mode) {
            case MIRROR -> new Translation2d(-t.getX(), t.getY());
            case ROTATE -> new Translation2d(-t.getX(), -t.getY());
            default -> t;
        };
    }

    // ================================================================ math helpers (3, 6.3)

    /** @return angle wrapped to (-pi, pi] */
    static double wrap(double angle) {
        double a = mod2pi(angle + Math.PI) - Math.PI;
        return a <= -Math.PI ? a + 2.0 * Math.PI : a;
    }

    /** @return angle wrapped to [0, 2pi) */
    static double mod2pi(double angle) {
        double twoPi = 2.0 * Math.PI;
        double a = angle % twoPi;
        if (a < 0) {
            a += twoPi;
        }
        return a >= twoPi ? 0.0 : a;   // a tiny negative angle rounds to 2pi after the add
    }

    /** @return z component of a x b */
    static double cross(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    /** @return sqrt(max(0, x)) */
    static double vsqrt(double x) {
        return Math.sqrt(Math.max(0.0, x));
    }

    /** @return the short way around (6.3): wrapped to (-pi, pi]; within HALF_TURN_TIE of a half turn is +pi (CCW) */
    static double shortTurn(double d) {
        double w = wrap(d);
        return Math.abs(w) > Math.PI - HALF_TURN_TIE ? Math.PI : w;
    }

    /** @return v rounded to 4 decimals, without trailing zeros */
    static String fmt(double v) {
        if (!Double.isFinite(v)) {
            return Double.toString(v);
        }
        return BigDecimal.valueOf(v).setScale(4, RoundingMode.HALF_UP).stripTrailingZeros().toPlainString();
    }

    // ================================================================ builder (5.3 to 5.8, 5.15)

    /** Feasibility conditions of section 5.7.1. */
    private enum Check { F1, F2, F3 }

    /** Collects build warnings as {@code Path "name", point i: text}, without duplicates (9.12.2). */
    private record Warnings(String pathName, List<String> out) {
        void add(int pointIndex, String text) {
            String message = "Path \"" + pathName + "\"" + (pointIndex >= 0 ? ", point " + pointIndex : "") + ": "
                    + text;
            if (!out.contains(message)) {
                out.add(message);
            }
        }
    }

    /** One point after merging duplicates; attributes merged, later value wins (5.8.2). */
    private static final class Node {
        final int userIndex;
        final double x;
        final double y;
        Rotation2d angle;
        double maxVelocity;
        double radius;
        boolean stop;
        double dwell;
        int speedFrom;     // user index that set the stop / speed attribute
        int radiusFrom;    // user index that set the radius

        Node(int userIndex, Point p) {
            this.userIndex = userIndex;
            this.speedFrom = userIndex;
            this.radiusFrom = userIndex;
            this.x = p.x();
            this.y = p.y();
            this.angle = p.angle().orElse(null);
            this.maxVelocity = p.maxVelocity().orElse(Double.NaN);
            this.radius = p.radius().orElse(Double.NaN);
            this.stop = p.isStop();
            this.dwell = p.dwellSeconds();
        }

        boolean hasSpeed() {
            return stop || !Double.isNaN(maxVelocity);
        }

        boolean hasRadius() {
            return !Double.isNaN(radius);
        }
    }

    /** Lines and arcs of one construction pass (5.4, 5.5) with the failed checks per corner. */
    private static final class Construction {
        final boolean[] lineExists;
        final double[] qsx;
        final double[] qsy;
        final double[] tx;
        final double[] ty;
        final double[] lambda;
        final double[] arcPhi0;
        final double[] arcDphi;
        final double[] arcWin;
        final List<Set<Check>> failed = new ArrayList<>();

        Construction(int n) {
            lineExists = new boolean[n - 1];
            qsx = new double[n - 1];
            qsy = new double[n - 1];
            tx = new double[n - 1];
            ty = new double[n - 1];
            lambda = new double[n - 1];
            arcPhi0 = new double[n];
            arcDphi = new double[n];
            arcWin = new double[n];
            for (int i = 0; i < n; i++) {
                failed.add(EnumSet.noneOf(Check.class));
            }
        }

        boolean anyFailed() {
            return failed.stream().anyMatch(f -> !f.isEmpty());
        }
    }

    /** Same as {@link #build(List, String, List, int)} with the normal shrink-pass guard. */
    static PathGeometry build(List<Point> points, String pathName, List<String> warnings) {
        return build(points, pathName, warnings, MAX_ITER);
    }

    /**
     * Runs the whole construction (section 5.15). Throws V1, V6, V30; adds W1 to W5 to warnings.
     *
     * @param points   the user's points, validated non-null
     * @param pathName the name used in the warning prefix
     * @param warnings receives build warnings
     * @param maxIter  shrink-pass guard (tests force the fallback)
     * @return the table; {@code waypointS} is indexed by the user's point index
     */
    static PathGeometry build(List<Point> points, String pathName, List<String> warnings, int maxIter) {
        Warnings w = new Warnings(pathName, warnings);
        int userCount = points.size();
        if (userCount < 2) {
            throw new IllegalArgumentException("Path needs at least 2 points (a start and an end), but you gave "
                    + userCount + ". Example: new Path(new Point(1, 2), new Point(3, 4)).");
        }
        for (int i = 0; i < userCount; i++) {
            Point p = points.get(i);
            if (p.isStop() && p.radius().isPresent() && p.radius().getAsDouble() > 0) {
                throw stopWithRadius(i, p.radius().getAsDouble());
            }
        }

        // 5.8.2: merge neighbors closer than EPS_DUP into the earlier point
        List<Node> merged = new ArrayList<>();
        int[] userToMerged = new int[userCount];
        for (int u = 0; u < userCount; u++) {
            Point p = points.get(u);
            if (!merged.isEmpty()) {
                Node kept = merged.get(merged.size() - 1);
                double d = Math.hypot(p.x() - kept.x, p.y() - kept.y);
                if (d < EPS_DUP) {
                    mergeInto(kept, u, p, d, w);
                    userToMerged[u] = merged.size() - 1;
                    continue;
                }
            }
            merged.add(new Node(u, p));
            userToMerged[u] = merged.size() - 1;
        }
        if (merged.size() == 1) {
            throw new IllegalArgumentException("All " + userCount + " points of the path are at the same place "
                    + "(within 1 mm), so the path has no length. Check the coordinates: two different points are "
                    + "needed, for example new Path(new Point(1, 2), new Point(3, 4)).");
        }
        for (Node node : merged) {
            if (node.stop && node.hasRadius() && node.radius > 0) {
                throw node.speedFrom == node.radiusFrom ? stopWithRadius(node.speedFrom, node.radius)
                        : mergedStopWithRadius(node.speedFrom, node.radiusFrom, node.radius);
            }
        }
        warnEndpoints(merged.get(0), merged.get(merged.size() - 1), w);

        // 5.8.3: set straight middle points aside as PASS markers
        List<Integer> geo = new ArrayList<>();
        for (int m = 0; m < merged.size(); m++) {
            geo.add(m);
        }
        List<Integer> passMarkers = new ArrayList<>();
        boolean changed = true;
        while (changed) {
            changed = false;
            for (int g = 1; g < geo.size() - 1; g++) {
                Node node = merged.get(geo.get(g));
                if (node.stop) {
                    continue;
                }
                Node prev = merged.get(geo.get(g - 1));
                Node next = merged.get(geo.get(g + 1));
                if (turnAngle(prev.x, prev.y, node.x, node.y, next.x, next.y) < EPS_ANG) {
                    passMarkers.add(geo.remove(g));
                    g--;
                    changed = true;
                }
            }
        }
        for (int m : passMarkers) {
            Node node = merged.get(m);
            if (node.hasRadius()) {
                w.add(node.userIndex, "withRadius(" + fmt(node.radius) + ") is ignored here: only a point "
                        + "where the path turns has a corner radius. Remove withRadius from this point.");
            }
        }

        int n = geo.size();
        double[] px = new double[n];
        double[] py = new double[n];
        Node[] nodes = new Node[n];
        for (int g = 0; g < n; g++) {
            nodes[g] = merged.get(geo.get(g));
            px[g] = nodes[g].x;
            py[g] = nodes[g].y;
        }
        double[] legLength = new double[n - 1];
        double[] divisor = new double[n - 1];
        for (int j = 0; j < n - 1; j++) {
            legLength[j] = Math.hypot(px[j + 1] - px[j], py[j + 1] - py[j]);
            boolean endLeg = j == 0 || j + 1 == n - 1 || nodes[j].stop || nodes[j + 1].stop;
            divisor[j] = endLeg ? RADIUS_DIVISOR_END : RADIUS_DIVISOR_SHARED;
        }

        // 5.3: corner construction
        double[] tau = new double[n];
        double[] sigma = new double[n];
        double[] bx = new double[n];
        double[] by = new double[n];
        double[] rawSigma = new double[n];
        for (int i = 1; i < n - 1; i++) {
            double inX = (px[i] - px[i - 1]) / legLength[i - 1];
            double inY = (py[i] - py[i - 1]) / legLength[i - 1];
            double outX = (px[i + 1] - px[i]) / legLength[i];
            double outY = (py[i + 1] - py[i]) / legLength[i];
            double cr = cross(inX, inY, outX, outY);
            tau[i] = Math.atan2(Math.abs(cr), inX * outX + inY * outY);
            rawSigma[i] = Math.signum(cr);
            double sx = -inX + outX;
            double sy = -inY + outY;
            double norm = Math.hypot(sx, sy);
            if (tau[i] > Math.PI - EPS_ANG || norm < 1e-12) {
                bx[i] = -inX;
                by[i] = -inY;
            } else {
                bx[i] = sx / norm;
                by[i] = sy / norm;
            }
        }
        for (int i = 1; i < n - 1; i++) {
            sigma[i] = rawSigma[i];
            if (!nodes[i].stop && tau[i] > Math.PI - EPS_ANG) {
                sigma[i] = uTurnSign(i, n, tau, rawSigma);
            }
        }

        // 5.7.2, 5.7.3: initial radii
        double[] r = new double[n];
        double[] r0 = new double[n];
        boolean[] override = new boolean[n];
        for (int i = 1; i < n - 1; i++) {
            Node node = nodes[i];
            if (node.stop) {
                r[i] = 0;
            } else if (node.hasRadius()) {
                override[i] = true;
                if (node.radius >= R_MIN) {
                    r[i] = node.radius;
                } else {
                    r[i] = 0;
                    if (node.radius > 0) {
                        w.add(node.userIndex, stopText("withRadius(" + fmt(node.radius) + ") is below "
                                + fmt(R_MIN) + " m"));
                    }
                }
            } else {
                double auto = Math.min(R_MAX, Math.min(legLength[i - 1] / divisor[i - 1], legLength[i] / divisor[i]));
                if (tau[i] >= EPS_ANG) {
                    auto = Math.min(auto, D_BULGE / (1.0 - Math.cos(tau[i] / 2.0)));
                }
                if (tau[i] > TAU_STOP) {
                    r[i] = 0;
                    w.add(node.userIndex, stopText("the path turns " + fmt(Math.toDegrees(tau[i]))
                            + " degrees here, more than " + fmt(Math.toDegrees(TAU_STOP)) + " degrees"));
                } else if (auto < R_MIN) {
                    r[i] = 0;
                    w.add(node.userIndex, stopText("the legs are too short: the largest radius that fits is "
                            + fmt(auto) + " m, less than " + fmt(R_MIN) + " m"));
                } else {
                    r[i] = auto;
                }
            }
            r0[i] = r[i];
        }

        // 5.7.3: shrink loop
        List<Set<Check>> shrinkReasons = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            shrinkReasons.add(EnumSet.noneOf(Check.class));
        }
        boolean[] fallbackZeroed = new boolean[n];
        Construction c = construct(n, px, py, r, sigma, tau, bx, by, legLength);
        int iter = 0;
        while (c.anyFailed()) {
            if (++iter > maxIter) {
                // hard guard (5.7.3): the polyline with stops always passes F1 to F3
                for (int i = 1; i < n - 1; i++) {
                    if (r[i] > 0) {
                        shrinkReasons.get(i).addAll(c.failed.get(i));
                        fallbackZeroed[i] = true;
                        r[i] = 0;
                    }
                }
                c = construct(n, px, py, r, sigma, tau, bx, by, legLength);
                break;
            }
            boolean anyAutomatic = false;
            for (int i = 1; i < n - 1; i++) {
                if (!c.failed.get(i).isEmpty() && !override[i]) {
                    anyAutomatic = true;
                }
            }
            for (int i = 1; i < n - 1; i++) {
                if (c.failed.get(i).isEmpty() || (anyAutomatic && override[i])) {
                    continue;
                }
                shrinkReasons.get(i).addAll(c.failed.get(i));
                r[i] *= SHRINK;
                if (r[i] < R_MIN) {
                    r[i] = 0;
                }
            }
            c = construct(n, px, py, r, sigma, tau, bx, by, legLength);
        }
        for (int i = 1; i < n - 1; i++) {
            if (fallbackZeroed[i]) {
                String why = shrinkReasons.get(i).isEmpty() ? "" : ", because " + checkList(shrinkReasons.get(i));
                w.add(nodes[i].userIndex, stopText("no radius fitted within " + maxIter
                        + " shrink passes, so every corner arc was removed" + why));
                continue;
            }
            if (shrinkReasons.get(i).isEmpty()) {
                continue;
            }
            String why = checkList(shrinkReasons.get(i));
            if (r[i] == 0) {
                w.add(nodes[i].userIndex, stopText("radius " + fmt(r0[i]) + " m was shrunk below "
                        + fmt(R_MIN) + " m to fit, because " + why));
            } else {
                w.add(nodes[i].userIndex, "radius " + fmt(r0[i]) + " m does not fit and was shrunk to "
                        + fmt(r[i]) + " m because " + why + ". Move the points farther apart, or give a smaller"
                        + " radius, for example withRadius(" + fmt(r[i]) + ").");
            }
        }

        // 5.9.1: segment table
        List<Segment> segments = new ArrayList<>();
        double[] geoS = new double[n];
        double s = 0;
        for (int j = 0; j < n - 1; j++) {
            if (j > 0) {
                int i = j;
                if (r[i] > 0) {
                    Segment arc = Segment.arc(s, new Translation2d(px[i] + r[i] * bx[i], py[i] + r[i] * by[i]), r[i],
                            c.arcPhi0[i], c.arcDphi[i], s + r[i] * c.arcWin[i]);
                    segments.add(arc);
                    geoS[i] = arc.sWp();
                    s += arc.length();
                } else {
                    segments.add(Segment.stop(s, new Translation2d(px[i], py[i]),
                            new Translation2d(c.tx[j - 1], c.ty[j - 1]), new Translation2d(c.tx[j], c.ty[j]),
                            nodes[i].userIndex, nodes[i].stop, nodes[i].stop ? nodes[i].dwell : 0.0));
                    geoS[i] = s;
                }
            }
            double len = c.lambda[j] < EPS_LEN ? 0.0 : c.lambda[j];
            segments.add(Segment.line(s, len, new Translation2d(c.qsx[j], c.qsy[j]),
                    new Translation2d(c.tx[j], c.ty[j])));
            s += len;
        }
        geoS[n - 1] = s;

        // 5.9.4: s of every merged point; PASS markers by projection between their kept neighbors
        double[] mergedS = new double[merged.size()];
        int[] geoOfMerged = new int[merged.size()];
        Arrays.fill(geoOfMerged, -1);
        for (int g = 0; g < n; g++) {
            mergedS[geo.get(g)] = geoS[g];
            geoOfMerged[geo.get(g)] = g;
        }
        PathGeometry provisional = new PathGeometry(segments, new double[userCount]);
        for (int m = 0; m < merged.size(); m++) {
            if (geoOfMerged[m] >= 0) {
                continue;
            }
            int prev = m - 1;
            while (geoOfMerged[prev] < 0) {
                prev--;
            }
            int next = m + 1;
            while (geoOfMerged[next] < 0) {
                next++;
            }
            Node node = merged.get(m);
            Translation2d p = new Translation2d(node.x, node.y);
            double sPass = provisional.project(p, mergedS[prev], mergedS[next], mergedS[prev]).s();
            mergedS[m] = sPass;
            double miss = p.getDistance(provisional.evaluate(sPass).position());
            if (miss > PASS_TOL) {
                w.add(node.userIndex, "this point is on a straight line between its neighbors, so it is "
                        + "only a pass marker, and the path misses it by " + fmt(miss) + " m (the line next to a "
                        + "round corner is turned outward). Add .stop() to drive exactly through it, or remove the "
                        + "point and use the command's progress() in a Trigger instead.");
            }
        }
        double[] waypointS = new double[userCount];
        for (int u = 0; u < userCount; u++) {
            waypointS[u] = mergedS[userToMerged[u]];
        }
        return new PathGeometry(segments, waypointS);
    }

    /** One construction pass: tangent lines (5.4), arc sweeps (5.5), checks F1 to F3 (5.7.1). */
    private static Construction construct(int n, double[] px, double[] py, double[] r, double[] sigma,
                                          double[] tau, double[] bx, double[] by, double[] legLength) {
        Construction c = new Construction(n);
        double[] cx = new double[n];
        double[] cy = new double[n];
        double[] er = new double[n];
        double[] es = new double[n];
        for (int i = 0; i < n; i++) {
            boolean circle = i > 0 && i < n - 1 && r[i] > 0;
            er[i] = circle ? r[i] : 0.0;
            es[i] = circle ? sigma[i] : 0.0;
            cx[i] = px[i] + er[i] * bx[i];
            cy[i] = py[i] + er[i] * by[i];
        }
        double[] qex = new double[n - 1];
        double[] qey = new double[n - 1];
        for (int j = 0; j < n - 1; j++) {
            double dx = cx[j + 1] - cx[j];
            double dy = cy[j + 1] - cy[j];
            double dc = Math.hypot(dx, dy);
            double h = es[j + 1] * er[j + 1] - es[j] * er[j];
            if (!(dc - Math.abs(h) > EPS_LEN)) {
                markCorners(c, j, n, er, Check.F1);
                continue;
            }
            double lam = Math.sqrt((dc - h) * (dc + h));
            double ux = dx / dc;
            double uy = dy / dc;
            double nx = (h * ux - lam * uy) / dc;
            double ny = (h * uy + lam * ux) / dc;
            double tx = (lam * ux + h * uy) / dc;
            double ty = (lam * uy - h * ux) / dc;
            c.lineExists[j] = true;
            c.lambda[j] = lam;
            c.tx[j] = tx;
            c.ty[j] = ty;
            c.qsx[j] = cx[j] - es[j] * er[j] * nx;
            c.qsy[j] = cy[j] - es[j] * er[j] * ny;
            qex[j] = cx[j + 1] - es[j + 1] * er[j + 1] * nx;
            qey[j] = cy[j + 1] - es[j + 1] * er[j + 1] * ny;
            double legX = (px[j + 1] - px[j]) / legLength[j];
            double legY = (py[j + 1] - py[j]) / legLength[j];
            if (Math.atan2(Math.abs(cross(tx, ty, legX, legY)), tx * legX + ty * legY) > EPS_DEV) {
                markCorners(c, j, n, er, Check.F2);
            }
        }
        for (int i = 1; i < n - 1; i++) {
            if (er[i] == 0 || !c.lineExists[i - 1] || !c.lineExists[i]) {
                continue;
            }
            double phiIn = Math.atan2(qey[i - 1] - cy[i], qex[i - 1] - cx[i]);
            double phiP = Math.atan2(py[i] - cy[i], px[i] - cx[i]);
            double phiOut = Math.atan2(c.qsy[i] - cy[i], c.qsx[i] - cx[i]);
            double wIn = mod2pi(sigma[i] * (phiP - phiIn));
            if (wIn > 2.0 * Math.PI - EPS_WRAP) {
                wIn = 0;
            }
            double wOut = mod2pi(sigma[i] * (phiOut - phiP));
            if (wOut > 2.0 * Math.PI - EPS_WRAP) {
                wOut = 0;
            }
            double dphi = sigma[i] * (wIn + wOut);
            c.arcPhi0[i] = phiIn;
            c.arcDphi[i] = dphi;
            c.arcWin[i] = wIn;
            if (Math.abs(Math.abs(dphi) - tau[i]) > 2.0 * EPS_DEV) {
                c.failed.get(i).add(Check.F3);
            }
        }
        return c;
    }

    /** Marks the corners with an arc at both ends of line j as failing check f. */
    private static void markCorners(Construction c, int j, int n, double[] er, Check f) {
        for (int i = j; i <= j + 1; i++) {
            if (i > 0 && i < n - 1 && er[i] > 0) {
                c.failed.get(i).add(f);
            }
        }
    }

    /** @return the U-turn turn sign: nearest proper corner, the next one first; +1 if none (5.8.4) */
    private static double uTurnSign(int i, int n, double[] tau, double[] rawSigma) {
        for (int dist = 1; dist < n; dist++) {
            for (int k : new int[] {i + dist, i - dist}) {
                if (k >= 1 && k <= n - 2 && rawSigma[k] != 0 && tau[k] <= Math.PI - EPS_ANG) {
                    return rawSigma[k];
                }
            }
        }
        return 1.0;
    }

    /** @return tau_turn at b for the polyline a, b, c, in [0, pi] (5.3) */
    private static double turnAngle(double ax, double ay, double bx, double by, double cx, double cy) {
        double inX = bx - ax;
        double inY = by - ay;
        double outX = cx - bx;
        double outY = cy - by;
        return Math.atan2(Math.abs(cross(inX, inY, outX, outY)), inX * outX + inY * outY);
    }

    /** Merges user point p (index u) into kept, later attribute wins; warning W3 (5.8.2). */
    private static void mergeInto(Node kept, int u, Point p, double d, Warnings w) {
        StringBuilder text = new StringBuilder("points " + kept.userIndex + " and " + u + " are only "
                + fmt(d * 1000.0) + " mm apart. Point " + u + " was merged into point " + kept.userIndex
                + ". Remove one of them from your list, for example delete point " + u + ".");
        if (p.angle().isPresent()) {
            if (kept.angle != null) {
                text.append(both("withAngle", fmt(kept.angle.getDegrees()) + " deg",
                        fmt(p.angle().get().getDegrees()) + " deg"));
            }
            kept.angle = p.angle().get();
        }
        if (p.isStop() || p.maxVelocity().isPresent()) {
            if (kept.hasSpeed()) {
                text.append(both("a speed setting", speedText(kept.stop, kept.dwell, kept.maxVelocity),
                        speedText(p.isStop(), p.dwellSeconds(), p.maxVelocity().orElse(Double.NaN))));
            }
            kept.speedFrom = u;
            kept.stop = p.isStop();
            kept.dwell = p.dwellSeconds();
            kept.maxVelocity = p.maxVelocity().orElse(Double.NaN);
        }
        if (p.radius().isPresent()) {
            if (kept.hasRadius()) {
                text.append(both("withRadius", fmt(kept.radius), fmt(p.radius().getAsDouble())));
            }
            kept.radiusFrom = u;
            kept.radius = p.radius().getAsDouble();
        }
        w.add(kept.userIndex, text.toString());
    }

    /** @return the W3 extra text for an attribute both points set */
    private static String both(String attribute, String a, String b) {
        return " both set " + attribute + " (" + a + " and " + b + "); using the later one (" + b + ").";
    }

    /** @return "stop()", "stop(1.5)" or "withVelocity(2)" */
    private static String speedText(boolean stop, double dwell, double maxVelocity) {
        if (stop) {
            return dwell > 0 ? "stop(" + fmt(dwell) + ")" : "stop()";
        }
        return "withVelocity(" + fmt(maxVelocity) + ")";
    }

    /** Warning W4 for settings on the first and last point that have no effect (9.12.2). */
    private static void warnEndpoints(Node first, Node last, Warnings w) {
        String noRadius = ") is ignored here: only a point where the path turns has a corner radius. Remove"
                + " withRadius from this point.";
        if (first.stop) {
            if (first.dwell > 0) {
                w.add(first.userIndex, "stop(" + fmt(first.dwell) + ") on the first point does not change "
                        + "the path; the robot waits " + fmt(first.dwell) + " s before it starts. Remove the wait unless you want it.");
            } else {
                w.add(first.userIndex, "stop() is ignored here: the robot starts with its measured speed. Remove stop() from this point.");
            }
        } else if (!Double.isNaN(first.maxVelocity)) {
            w.add(first.userIndex, "withVelocity(" + fmt(first.maxVelocity) + ") is ignored here: the robot "
                    + "starts with its measured speed. Remove withVelocity from this point.");
        }
        if (first.hasRadius()) {
            w.add(first.userIndex, "withRadius(" + fmt(first.radius) + noRadius);
        }
        if (!Double.isNaN(last.maxVelocity)) {
            w.add(last.userIndex, "withVelocity(" + fmt(last.maxVelocity) + ") is ignored here: the robot "
                    + "always ends at speed 0 in this version. Remove withVelocity from the last point.");
        }
        if (last.hasRadius()) {
            w.add(last.userIndex, "withRadius(" + fmt(last.radius) + noRadius);
        }
    }

    /** @return the W1 text with its reason */
    private static String stopText(String reason) {
        return "the corner is too sharp or too tight for an arc (" + reason + "). The robot will stop here. To drive "
                + "through it with an arc, give a radius of at least " + fmt(R_MIN) + " m, for example "
                + "withRadius(0.3). To stop on purpose, use stop() or withRadius(0).";
    }

    /** @return "a and b" list of the failed checks */
    private static String checkList(Set<Check> checks) {
        StringJoiner joiner = new StringJoiner(" and ");
        for (Check check : checks) {
            joiner.add(switch (check) {
                case F1 -> "a neighbor point is too close to the circle";
                case F2 -> "the line would run more than 30 degrees off the straight leg";
                case F3 -> "the arc would turn much more than the corner angle";
            });
        }
        return joiner.toString();
    }

    /** @return the V30 exception for point i */
    private static IllegalArgumentException stopWithRadius(int i, double radius) {
        return new IllegalArgumentException("Point " + i + " has both stop() and withRadius(" + radius + "). A stop "
                + "point is a sharp corner (the robot stops exactly on the point), so it has no arc. Remove withRadius("
                + radius + ") to stop on the point, or remove stop() to drive through the corner on a round arc.");
    }

    /** @return the V30 exception when stop() and withRadius come from two merged points */
    private static IllegalArgumentException mergedStopWithRadius(int stopIndex, int radiusIndex, double radius) {
        return new IllegalArgumentException("Points " + Math.min(stopIndex, radiusIndex) + " and "
                + Math.max(stopIndex, radiusIndex) + " are less than 1 mm apart and were merged into one point, but "
                + "point " + stopIndex + " has stop() and point " + radiusIndex + " has withRadius(" + radius + "). A "
                + "stop point is a sharp corner (the robot stops exactly on the point), so it has no arc. Remove "
                + "withRadius(" + radius + ") from point " + radiusIndex + " to stop on the point, or remove stop() "
                + "from point " + stopIndex + " to drive through the corner on a round arc.");
    }
}
