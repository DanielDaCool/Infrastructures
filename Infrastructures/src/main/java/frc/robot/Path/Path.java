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

    /**
     * A waypoint the path is built around. radius 0: the path goes exactly through the point
     * (start, end, stop or sharp corner). radius > 0: a corner circle through the waypoint.
     */
    private record Corner(int index, Translation2d center, double radius, boolean isLeftTurn, double turn) {
        boolean isPoint() { return radius == 0; }

        /** + for a left turn, - for a right turn, 0 for a point (used by the tangent formula). */
        double signedRadius() { return isLeftTurn ? radius : -radius; }
    }

    private final Point[] points;
    private final List<Segment> segments = new ArrayList<>();
    private final List<Double> starts = new ArrayList<>();
    private final List<Stop> stops = new ArrayList<>();
    private final List<Anchor> anchors = new ArrayList<>();
    private double length;

    /**
     * Builds the path in four steps: check the points, turn each waypoint into a corner, join the
     * corners with tangent lines and arcs, then place the headings.
     *
     * @param points at least 2 waypoints, consecutive ones more than 1 cm apart
     */
    public Path(Point... points) {
        checkPoints(points);
        this.points = points.clone();

        List<Corner> corners = new ArrayList<>();
        for (int i = 0; i < points.length; i++) {
            Corner corner = cornerAt(i);
            if (corner != null) corners.add(corner);
        }

        double[] waypointS = new double[points.length]; // path distance of each waypoint
        Arrays.fill(waypointS, Double.NaN);
        buildSegments(corners, waypointS);

        for (int i = 1; i < points.length; i++) { // waypoints dropped as straight: closest path point
            if (Double.isNaN(waypointS[i])) waypointS[i] = locate(pos(i), waypointS[i - 1]);
        }
        for (int i = 0; i < points.length; i++) {
            if (points[i].angle() != null) anchors.add(new Anchor(waypointS[i], points[i].angle()));
        }
    }

    private static void checkPoints(Point[] points) {
        if (points == null || points.length < 2) throw new IllegalArgumentException("Path needs at least 2 points");
        for (int i = 0; i < points.length; i++) {
            if (points[i] == null) throw new IllegalArgumentException("Path point " + i + " is null");
            if (i > 0 && points[i].position().getDistance(points[i - 1].position()) <= 0.01)
                throw new IllegalArgumentException("Path points " + (i - 1) + " and " + i + " are within 1 cm");
        }
    }

    /** The corner for waypoint i, or null when the path runs straight through it (no turn). */
    private Corner cornerAt(int i) {
        Translation2d v = pos(i);
        boolean endOrStop = i == 0 || i == points.length - 1 || points[i].isStop();
        if (endOrStop) return new Corner(i, v, 0, false, 0);

        Translation2d in = unit(v.minus(pos(i - 1)));
        Translation2d out = unit(pos(i + 1).minus(v));
        double turn = Math.atan2(cross(in, out), in.dot(out)); // + left, - right
        if (Math.abs(turn) > MAX_ARC_TURN) return new Corner(i, v, 0, false, 0); // too sharp: stop here
        if (Math.abs(turn) < 1e-3) return null;

        double r = Double.isNaN(points[i].radius()) ? RADIUS : points[i].radius();
        // The center is on the inside of the corner, r from the waypoint, so the circle passes through it.
        Translation2d center = v.plus(unit(out.minus(in)).times(r));
        return new Corner(i, center, r, turn > 0, Math.abs(turn));
    }

    /** Adds the lines and arcs between the corners, the stops, and the path distance of each corner. */
    private void buildSegments(List<Corner> corners, double[] waypointS) {
        List<Translation2d[]> legs = new ArrayList<>(); // legs.get(j) joins corner j to corner j + 1
        for (int j = 0; j + 1 < corners.size(); j++) {
            Corner from = corners.get(j), to = corners.get(j + 1);
            Translation2d[] leg = tangentLeg(from, to);
            if (leg == null) throw arcError(to.isPoint() ? from.index() : to.index());
            legs.add(leg);
        }

        for (int j = 0; j < corners.size(); j++) {
            Corner corner = corners.get(j);
            if (corner.isPoint()) {
                waypointS[corner.index()] = length;
                if (j > 0) stops.add(new Stop(length, points[corner.index()].dwell()));
            } else {
                Arc arc = cornerArc(corner, legs.get(j - 1)[1], legs.get(j)[0]);
                waypointS[corner.index()] = length + MathUtil.clamp(arc.project(pos(corner.index())), 0, arc.length());
                add(arc);
            }
            if (j < legs.size()) {
                Translation2d[] leg = legs.get(j);
                if (leg[0].getDistance(leg[1]) > 1e-6) add(new Line(leg[0], leg[1]));
            }
        }
    }

    /** The arc of a corner circle from the incoming leg's end to the outgoing leg's start. */
    private Arc cornerArc(Corner corner, Translation2d entry, Translation2d exit) {
        double sign = corner.isLeftTurn() ? 1 : -1;
        double startAngle = polar(entry.minus(corner.center()));
        double sweep = mod2pi(sign * (polar(exit.minus(corner.center())) - startAngle));
        double toWaypoint = mod2pi(sign * (polar(pos(corner.index()).minus(corner.center())) - startAngle));
        // A valid arc turns a little more than the corner and passes its waypoint; anything else
        // (no arc, a loop around the circle, waypoint outside the arc) means the corners are too close.
        boolean valid = sweep > 1e-9 && sweep <= corner.turn() + Math.PI / 2 && toWaypoint <= sweep + 1e-9;
        if (!valid) throw arcError(corner.index());
        return new Arc(corner.center(), corner.radius(), corner.isLeftTurn(), startAngle, sweep);
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
     * The straight leg leaving corner "from" and arriving at corner "to", touching both circles
     * (a point is a circle of radius 0). Returns {start, end}, or null when the circles are too
     * close for such a leg.
     *
     * <p>With signed radii (+ left turn, - right turn), one formula covers all cases: m is the unit
     * normal on the leg's left, at angle acos((r2 - r1) / d) from the center-to-center direction, and
     * each tangent point is center - r * m.
     */
    private static Translation2d[] tangentLeg(Corner from, Corner to) {
        Translation2d delta = to.center().minus(from.center());
        double d = delta.getNorm(), q = to.signedRadius() - from.signedRadius();
        if (!(d > Math.abs(q) + 1e-9)) return null;
        Translation2d m = unit(delta).rotateBy(new Rotation2d(Math.acos(q / d)));
        return new Translation2d[] {
            from.center().minus(m.times(from.signedRadius())), to.center().minus(m.times(to.signedRadius()))
        };
    }

    private static IllegalArgumentException arcError(int i) {
        return new IllegalArgumentException(
                "Path point " + i + ": corner arc does not fit; move the points apart or use a smaller radius");
    }

    private Translation2d pos(int i) { return points[i].position(); }

    private static Translation2d unit(Translation2d v) { return v.div(v.getNorm()); }

    static double cross(Translation2d a, Translation2d b) { return a.getX() * b.getY() - a.getY() * b.getX(); }

    private static double polar(Translation2d v) { return Math.atan2(v.getY(), v.getX()); }

    private static double mod2pi(double a) { return MathUtil.inputModulus(a, 0, 2 * Math.PI); }
}
