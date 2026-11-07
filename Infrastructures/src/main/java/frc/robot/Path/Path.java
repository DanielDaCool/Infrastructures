package frc.robot.Path;

import static frc.robot.Path.PathConstants.MAX_ARC_TURN;
import static frc.robot.Path.PathConstants.RADIUS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Lines joined by corner arcs through the waypoints, in field coordinates [m]. Each interior
 * waypoint becomes a circle through it (tangent legs between circles), a sharp stop corner, or is
 * dropped when the path runs straight through it. The end is always a stop.
 */
public final class Path {
    record Stop(double s, double dwell) {}

    record Anchor(double s, Rotation2d angle) {}

    private final Point[] points;
    private final List<Segment> segments = new ArrayList<>();
    private final List<Double> starts = new ArrayList<>();
    private final List<Stop> stops = new ArrayList<>();
    private final List<Anchor> anchors = new ArrayList<>();
    private double length;

    /** @param points at least 2 waypoints, consecutive ones more than 1 cm apart */
    public Path(Point... points) {
        if (points == null || points.length < 2) throw new IllegalArgumentException("Path needs at least 2 points");
        for (int i = 0; i < points.length; i++) {
            if (points[i] == null) throw new IllegalArgumentException("Path point " + i + " is null");
            if (i > 0 && pos(points, i).getDistance(pos(points, i - 1)) <= 0.01)
                throw new IllegalArgumentException("Path points " + (i - 1) + " and " + i + " are within 1 cm");
        }
        this.points = points.clone();
        int n = points.length;
        Translation2d[] centers = new Translation2d[n];
        double[] rho = new double[n]; // sigma * r; 0 = point node
        double[] turn = new double[n]; // |theta| of circle nodes
        double[] sOf = new double[n];
        Arrays.fill(sOf, Double.NaN);
        List<Integer> nodes = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            Translation2d v = pos(points, i);
            centers[i] = v;
            if (i == 0 || i == n - 1 || points[i].isStop()) {
                nodes.add(i);
                continue;
            }
            Translation2d uIn = unit(v.minus(pos(points, i - 1)));
            Translation2d uOut = unit(pos(points, i + 1).minus(v));
            double theta = Math.atan2(cross(uIn, uOut), uIn.dot(uOut));
            if (Math.abs(theta) > MAX_ARC_TURN) {
                nodes.add(i);
            } else if (Math.abs(theta) >= 1e-3) {
                double r = Double.isNaN(points[i].radius()) ? RADIUS : points[i].radius();
                centers[i] = v.plus(unit(uOut.minus(uIn)).times(r));
                rho[i] = Math.signum(theta) * r;
                turn[i] = Math.abs(theta);
                nodes.add(i);
            }
        }
        Translation2d[][] legs = new Translation2d[nodes.size() - 1][];
        for (int j = 0; j < legs.length; j++) {
            int a = nodes.get(j), b = nodes.get(j + 1);
            legs[j] = tangent(centers[a], rho[a], centers[b], rho[b]);
            if (legs[j] == null) throw arcError(rho[b] != 0 ? b : a);
        }
        for (int j = 0; j < nodes.size(); j++) {
            int i = nodes.get(j);
            if (rho[i] != 0) {
                Translation2d c = centers[i];
                int sigma = (int) Math.signum(rho[i]);
                double a0 = polar(legs[j - 1][1].minus(c));
                double sweep = mod2pi(sigma * (polar(legs[j][0].minus(c)) - a0));
                double rel = mod2pi(sigma * (polar(pos(points, i).minus(c)) - a0));
                if (!(sweep > 1e-9 && sweep <= turn[i] + Math.PI / 2 && rel <= sweep + 1e-9)) throw arcError(i);
                Arc arc = new Arc(c, Math.abs(rho[i]), sigma > 0, a0, sweep);
                sOf[i] = length + Math.min(arc.length(), Math.abs(rho[i]) * rel);
                add(arc);
            } else {
                sOf[i] = length;
                if (i > 0) stops.add(new Stop(length, points[i].dwell()));
            }
            if (j < legs.length && legs[j][1].getDistance(legs[j][0]) > 1e-6) add(new Line(legs[j][0], legs[j][1]));
        }
        for (int i = 1; i < n; i++) if (Double.isNaN(sOf[i])) sOf[i] = locate(pos(points, i), sOf[i - 1]);
        for (int i = 0; i < n; i++) if (points[i].angle() != null) anchors.add(new Anchor(sOf[i], points[i].angle()));
    }

    /** This path for the red alliance, per {@link PathConstants#MIRRORED_FIELD}. */
    public Path flipped() {
        return flipped(PathConstants.MIRRORED_FIELD);
    }

    Path flipped(boolean mirror) {
        return new Path(Arrays.stream(points).map(p -> p.flipped(mirror)).toArray(Point[]::new));
    }

    /** Total length [m]. */
    public double length() { return length; }

    public Translation2d start() { return points[0].position(); }

    public Translation2d end() { return points[points.length - 1].position(); }

    List<Segment> segments() { return Collections.unmodifiableList(segments); }

    /** Path distance [m] at the start of segment k. */
    double segmentStart(int k) { return starts.get(k); }

    /** Stops in path order; the last one is the end. */
    List<Stop> stops() { return Collections.unmodifiableList(stops); }

    /** Heading anchors in path order. */
    List<Anchor> anchors() { return Collections.unmodifiableList(anchors); }

    /** Index of the segment containing path distance s [m]. */
    int indexAt(double s) {
        int k = segments.size() - 1;
        while (k > 0 && starts.get(k) > s) k--;
        return k;
    }

    Translation2d pointAt(double s) {
        int k = indexAt(s);
        return segments.get(k).pointAt(s - starts.get(k));
    }

    Translation2d tangentAt(double s) {
        int k = indexAt(s);
        return segments.get(k).tangentAt(s - starts.get(k));
    }

    private void add(Segment segment) {
        segments.add(segment);
        starts.add(length);
        length += segment.length();
    }

    /** Path distance [m] >= fromS of the path point closest to p. */
    private double locate(Translation2d p, double fromS) {
        double best = fromS, bestDistance = Double.POSITIVE_INFINITY;
        for (int k = 0; k < segments.size(); k++) {
            Segment seg = segments.get(k);
            double s = Math.max(fromS, starts.get(k) + MathUtil.clamp(seg.project(p), 0, seg.length()));
            if (s > starts.get(k) + seg.length()) continue;
            double d = pointAt(s).getDistance(p);
            if (d < bestDistance - 1e-9) {
                best = s;
                bestDistance = d;
            }
        }
        return best;
    }

    /**
     * Directed common tangent leaving circle 1 and reaching circle 2 (rho = sigma * r, 0 = point);
     * returns {T1, T2}, or null when it does not exist.
     */
    private static Translation2d[] tangent(Translation2d c1, double rho1, Translation2d c2, double rho2) {
        Translation2d delta = c2.minus(c1);
        double d = delta.getNorm(), q = rho2 - rho1;
        if (!(d > Math.abs(q) + 1e-9)) return null;
        Translation2d m = unit(delta).rotateBy(new Rotation2d(Math.acos(q / d))); // left normal of the leg
        return new Translation2d[] {c1.minus(m.times(rho1)), c2.minus(m.times(rho2))};
    }

    private static IllegalArgumentException arcError(int i) {
        return new IllegalArgumentException(
                "Path point " + i + ": corner arc does not fit; move the points apart or use a smaller radius");
    }

    private static Translation2d pos(Point[] points, int i) { return points[i].position(); }

    private static Translation2d unit(Translation2d v) { return v.div(v.getNorm()); }

    static double cross(Translation2d a, Translation2d b) { return a.getX() * b.getY() - a.getY() * b.getX(); }

    private static double polar(Translation2d v) { return Math.atan2(v.getY(), v.getX()); }

    private static double mod2pi(double a) { return MathUtil.inputModulus(a, 0, 2 * Math.PI); }
}
