package frc.robot.Path;

import static frc.robot.Path.PathConfig.*;

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
    /** Corners turning more than this become stops [rad]. */
    private static final double MAX_ARC_TURN = Math.toRadians(150);

    record Stop(double distance, double dwell) {}

    record Anchor(double distance, Rotation2d angle) {}

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
    private final double maxVelocity, maxAcceleration, maxOmega;
    private final List<Segment> segments = new ArrayList<>();
    private final List<Double> segmentStarts = new ArrayList<>();
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
        this(points, MAX_VELOCITY, MAX_ACCELERATION, MAX_OMEGA);
    }

    private Path(Point[] points, double maxVelocity, double maxAcceleration, double maxOmega) {
        checkPoints(points);
        this.points = points.clone();
        this.maxVelocity = checkLimit(maxVelocity, "max velocity");
        this.maxAcceleration = checkLimit(maxAcceleration, "max acceleration");
        this.maxOmega = checkLimit(maxOmega, "max omega");

        List<Corner> corners = new ArrayList<>();
        for (int i = 0; i < points.length; i++) {
            Corner corner = cornerAt(i);
            if (corner != null) corners.add(corner);
        }

        double[] waypointDistance = new double[points.length]; // path distance of each waypoint
        Arrays.fill(waypointDistance, Double.NaN);
        buildSegments(corners, waypointDistance);

        for (int i = 1; i < points.length; i++) { // waypoints dropped as straight: closest path point
            if (Double.isNaN(waypointDistance[i])) waypointDistance[i] = locate(waypointPosition(i), waypointDistance[i - 1]);
        }
        for (int i = 0; i < points.length; i++) {
            if (points[i].angle() != null) anchors.add(new Anchor(waypointDistance[i], points[i].angle()));
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

    /** The corner for waypoint index, or null when the path runs straight through it (no turn). */
    private Corner cornerAt(int index) {
        Translation2d waypoint = waypointPosition(index);
        boolean endOrStop = index == 0 || index == points.length - 1 || points[index].isStop();
        if (endOrStop) return new Corner(index, waypoint, 0, false, 0);

        Translation2d directionIn = unitVector(waypoint.minus(waypointPosition(index - 1)));
        Translation2d directionOut = unitVector(waypointPosition(index + 1).minus(waypoint));
        double turn = Math.atan2(cross(directionIn, directionOut), directionIn.dot(directionOut)); // + left, - right
        if (Math.abs(turn) > MAX_ARC_TURN) return new Corner(index, waypoint, 0, false, 0); // too sharp: stop here
        if (Math.abs(turn) < 1e-3) return null;

        double radius = Double.isNaN(points[index].radius()) ? RADIUS : points[index].radius();
        // The center is on the inside of the corner, radius from the waypoint, so the circle passes through it.
        Translation2d center = waypoint.plus(unitVector(directionOut.minus(directionIn)).times(radius));
        return new Corner(index, center, radius, turn > 0, Math.abs(turn));
    }

    /** Adds the lines and arcs between the corners, the stops, and the path distance of each corner. */
    private void buildSegments(List<Corner> corners, double[] waypointDistance) {
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
                waypointDistance[corner.index()] = length;
                if (j > 0) stops.add(new Stop(length, points[corner.index()].dwell()));
            } else {
                Arc arc = cornerArc(corner, legs.get(j - 1)[1], legs.get(j)[0]);
                waypointDistance[corner.index()] = length + MathUtil.clamp(arc.project(waypointPosition(corner.index())), 0, arc.length());
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
        double startAngle = angleOf(entry.minus(corner.center()));
        double sweep = wrapTo2Pi(sign * (angleOf(exit.minus(corner.center())) - startAngle));
        double toWaypoint = wrapTo2Pi(sign * (angleOf(waypointPosition(corner.index()).minus(corner.center())) - startAngle));
        // A valid arc turns a little more than the corner and passes its waypoint; anything else
        // (no arc, a loop around the circle, waypoint outside the arc) means the corners are too close.
        boolean valid = sweep > 1e-9 && sweep <= corner.turn() + Math.PI / 2 && toWaypoint <= sweep + 1e-9;
        if (!valid) throw arcError(corner.index());
        return new Arc(corner.center(), corner.radius(), corner.isLeftTurn(), startAngle, sweep);
    }

    /**
     * This path with its own limits instead of the PathConfig ones.
     *
     * @param maxVelocity [m/s] @param maxAcceleration [m/s^2] @param maxOmega [rad/s], each > 0
     */
    public Path withLimits(double maxVelocity, double maxAcceleration, double maxOmega) {
        return new Path(points, maxVelocity, maxAcceleration, maxOmega);
    }

    /** Top speed along this path [m/s]. */ public double maxVelocity() { return maxVelocity; }
    /** Acceleration along this path [m/s^2]. */ public double maxAcceleration() { return maxAcceleration; }
    /** Top turn rate on this path [rad/s]. */ public double maxOmega() { return maxOmega; }

    private static double checkLimit(double value, String name) {
        if (!(Double.isFinite(value) && value > 0)) throw new IllegalArgumentException("Path " + name + " must be > 0");
        return value;
    }

    /** This path for the red alliance, per {@link PathConfig#MIRRORED_FIELD}. */
    public Path flipped() {
        return flipped(PathConfig.MIRRORED_FIELD);
    }

    Path flipped(boolean mirror) {
        return new Path(Arrays.stream(points).map(point -> point.flipped(mirror)).toArray(Point[]::new),
                maxVelocity, maxAcceleration, maxOmega);
    }

    /** Total length [m]. */
    public double length() { return length; }

    public Translation2d start() { return points[0].position(); }

    public Translation2d end() { return points[points.length - 1].position(); }

    List<Segment> segments() { return Collections.unmodifiableList(segments); }

    /** Path distance [m] at the start of segment index. */
    double segmentStart(int index) { return segmentStarts.get(index); }

    /** Stops in path order; the last one is the end. */
    List<Stop> stops() { return Collections.unmodifiableList(stops); }

    /** Heading anchors in path order. */
    List<Anchor> anchors() { return Collections.unmodifiableList(anchors); }

    /** Index of the segment containing path distance [m]. */
    int indexAt(double distance) {
        int index = segments.size() - 1;
        while (index > 0 && segmentStarts.get(index) > distance) index--;
        return index;
    }

    Translation2d pointAt(double distance) {
        int index = indexAt(distance);
        return segments.get(index).pointAt(distance - segmentStarts.get(index));
    }

    Translation2d tangentAt(double distance) {
        int index = indexAt(distance);
        return segments.get(index).tangentAt(distance - segmentStarts.get(index));
    }

    private void add(Segment segment) {
        segments.add(segment);
        segmentStarts.add(length);
        length += segment.length();
    }

    /** Path distance [m] >= fromDistance of the path point closest to point. */
    private double locate(Translation2d point, double fromDistance) {
        double best = fromDistance, bestDistance = Double.POSITIVE_INFINITY;
        for (int index = 0; index < segments.size(); index++) {
            Segment segment = segments.get(index);
            double distance = Math.max(fromDistance,
                    segmentStarts.get(index) + MathUtil.clamp(segment.project(point), 0, segment.length()));
            if (distance > segmentStarts.get(index) + segment.length()) continue;
            double gap = pointAt(distance).getDistance(point);
            if (gap < bestDistance - 1e-9) {
                best = distance;
                bestDistance = gap;
            }
        }
        return best;
    }

    /**
     * The straight leg leaving corner "from" and arriving at corner "to", touching both circles
     * (a point is a circle of radius 0). Returns {start, end}, or null when the circles are too
     * close for such a leg.
     *
     * <p>With signed radii (+ left turn, - right turn), one formula covers all cases: legNormal is the unit
     * normal on the leg's left, at angle acos(radiusDifference / centerDistance) from the center-to-center direction, and
     * each tangent point is center - radius * legNormal.
     */
    private static Translation2d[] tangentLeg(Corner from, Corner to) {
        Translation2d centerToCenter = to.center().minus(from.center());
        double centerDistance = centerToCenter.getNorm();
        double radiusDifference = to.signedRadius() - from.signedRadius();
        if (!(centerDistance > Math.abs(radiusDifference) + 1e-9)) return null;
        Translation2d legNormal =
                unitVector(centerToCenter).rotateBy(new Rotation2d(Math.acos(radiusDifference / centerDistance)));
        return new Translation2d[] {
            from.center().minus(legNormal.times(from.signedRadius())), to.center().minus(legNormal.times(to.signedRadius()))
        };
    }

    private static IllegalArgumentException arcError(int pointIndex) {
        return new IllegalArgumentException(
                "Path point " + pointIndex + ": corner arc does not fit; move the points apart or use a smaller radius");
    }

    private Translation2d waypointPosition(int index) { return points[index].position(); }

    private static Translation2d unitVector(Translation2d vector) { return vector.div(vector.getNorm()); }

    static double cross(Translation2d a, Translation2d b) { return a.getX() * b.getY() - a.getY() * b.getX(); }

    private static double angleOf(Translation2d vector) { return Math.atan2(vector.getY(), vector.getX()); }

    private static double wrapTo2Pi(double angle) { return MathUtil.inputModulus(angle, 0, 2 * Math.PI); }
}
