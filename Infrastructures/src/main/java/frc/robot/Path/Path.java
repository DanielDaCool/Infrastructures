package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Points, constraints, face target and events of one path. Builds and holds the geometry
 * (blue frame) and memoizes the flipped copy (design doc section 9.4). Immutable: each
 * {@code withX} returns a new Path that shares the geometry.
 */
public final class Path {

    private static final AtomicInteger COUNTER = new AtomicInteger();

    private final List<Point> points;
    private final SegmentTable table;
    private final PathConstraints constraints;   // null = resolved when the command starts
    private final Translation2d faceTarget;       // null = none
    private final List<ResolvedEvent> events;
    private final String name;
    private final List<String> warnings;
    private final Path blueSource;                // null unless this is a flipped copy
    private final FlipKey flipKey;                // null unless this is a flipped copy
    private final Map<FlipKey, Path> flipCache;   // used on blue paths only

    /** Memo key of a flip: mode and field size. */
    private record FlipKey(FlipMode mode, double fieldLength, double fieldWidth) {
    }

    /**
     * Builds the geometry. Throws on bad input (V1, V3, V6, V30), warns for repairs.
     *
     * @param points two or more points
     */
    public Path(Point... points) {
        this(null, checkPoints("new Path", points), false);
    }

    /**
     * @param constraints per-path constraints
     * @param points      two or more points
     */
    public Path(PathConstraints constraints, Point... points) {
        this(Objects.requireNonNull(constraints, Point.nullMessage("new Path", "the constraints")),
                checkPoints("new Path", points), false);
    }

    /** Builds from a checked list. */
    private Path(PathConstraints constraints, List<Point> points, boolean unused) {
        this.points = points;
        this.name = "path" + COUNTER.incrementAndGet();
        PathWarnings collector = new PathWarnings(name);
        this.table = GeometryBuilder.build(points, collector);
        this.warnings = collector.list();
        this.constraints = constraints;
        this.faceTarget = null;
        this.events = List.of();
        this.blueSource = null;
        this.flipKey = null;
        this.flipCache = new HashMap<>();
    }

    /** Copy constructor: shares the geometry. A flipped copy has a blue source and its flip key. */
    private Path(List<Point> points, SegmentTable table, PathConstraints constraints, Translation2d faceTarget,
                 List<ResolvedEvent> events, String name, List<String> warnings, Path blueSource,
                 FlipKey flipKey) {
        this.points = points;
        this.table = table;
        this.constraints = constraints;
        this.faceTarget = faceTarget;
        this.events = events;
        this.name = name;
        this.warnings = warnings;
        this.blueSource = blueSource;
        this.flipKey = flipKey;
        this.flipCache = new HashMap<>();
    }

    /** @return a blue copy with new values; geometry and points shared */
    private Path blueCopy(PathConstraints c, Translation2d target, List<ResolvedEvent> evts, String n,
                          List<String> w) {
        return new Path(points, table, c, target, evts, n, w, null, null);
    }

    /** @return a path through the points */
    public static Path of(List<Point> points) {
        Objects.requireNonNull(points, Point.nullMessage("Path.of", "the points list"));
        return new Path(points.toArray(new Point[0]));
    }

    /** @return a path through the points with per-path constraints */
    public static Path of(PathConstraints constraints, List<Point> points) {
        Objects.requireNonNull(constraints, Point.nullMessage("Path.of", "the constraints"));
        Objects.requireNonNull(points, Point.nullMessage("Path.of", "the points list"));
        return new Path(constraints, points.toArray(new Point[0]));
    }

    /**
     * The name is the telemetry key only: warning texts keep the automatic name the path had when
     * it was built ({@code Path "path3", ...}); withName does not rewrite them.
     *
     * @return a copy with a name (telemetry key {@code Path/<name>}); on a flipped copy, the same
     *         flip of the renamed blue path
     */
    public Path withName(String name) {
        Objects.requireNonNull(name, Point.nullMessage("withName", "the name"));
        if (blueSource != null) {
            return blueSource.withName(name).flippedBy(flipKey);
        }
        return blueCopy(constraints, faceTarget, events, name, warnings);
    }

    /** @return a copy with per-path constraints; on a flipped copy, the same flip of the changed blue path */
    public Path withConstraints(PathConstraints constraints) {
        Objects.requireNonNull(constraints, Point.nullMessage("withConstraints", "the constraints"));
        if (blueSource != null) {
            return blueSource.withConstraints(constraints).flippedBy(flipKey);
        }
        return blueCopy(constraints, faceTarget, events, name, warnings);
    }

    /**
     * The target is in the blue frame, also on a flipped copy (it is flipped with the path).
     *
     * @return a copy whose robot front faces the target
     */
    public Path withFaceTarget(Translation2d target) {
        Objects.requireNonNull(target, Point.nullMessage("withFaceTarget", "the face target"));
        if (blueSource != null) {
            return blueSource.withFaceTarget(target).flippedBy(flipKey);
        }
        if (!Double.isFinite(target.getX()) || !Double.isFinite(target.getY())) {
            throw new IllegalArgumentException(Point.notFiniteMessage("withFaceTarget",
                    Double.isFinite(target.getX()) ? target.getY() : target.getX(), "the face target", "meters"));
        }
        PathWarnings extra = new PathWarnings(name);
        for (int i = 0; i < points.size(); i++) {
            if (points.get(i).angle().isPresent()) {
                extra.add(i, "withAngle is ignored here: the robot looks at the face target ("
                        + GeometryBuilder.fmt(target.getX()) + ", " + GeometryBuilder.fmt(target.getY())
                        + "); remove withAngle or use withoutFaceTarget().");
            }
        }
        List<String> all = new ArrayList<>(warnings);
        for (String w : extra.list()) {
            if (!all.contains(w)) {
                all.add(w);
            }
        }
        return blueCopy(constraints, target, events, name, Collections.unmodifiableList(all));
    }

    /** @return a copy without a face target; on a flipped copy, the same flip of the changed blue path */
    public Path withoutFaceTarget() {
        if (blueSource != null) {
            return blueSource.withoutFaceTarget().flippedBy(flipKey);
        }
        return blueCopy(constraints, null, events, name, warnings);
    }

    /**
     * @return a copy with the event, resolved to s now (5.9.5); on a flipped copy, the same flip of
     *         the changed blue path
     */
    public Path withEvent(PathEvent event) {
        Objects.requireNonNull(event, Point.nullMessage("withEvent", "the event"));
        if (blueSource != null) {
            return blueSource.withEvent(event).flippedBy(flipKey);
        }
        double s;
        if (event.kind() == PathEvent.Kind.FRACTION) {
            double f = event.value();
            if (!(f >= 0.0 && f <= 1.0)) {
                throw new IllegalArgumentException("Event fraction " + f
                        + " must be between 0 and 1 (0 = start of the path, 1 = end).");
            }
            s = f * table.length();
        } else {
            s = table.waypointS(checkIndex((int) event.value()));
        }
        List<ResolvedEvent> list = new ArrayList<>(events);
        list.add(new ResolvedEvent(s, event));
        return blueCopy(constraints, faceTarget, Collections.unmodifiableList(list), name, warnings);
    }

    /** @return a copy with an event at fraction f of the length */
    public Path withEventAtFraction(double fraction, Command action) {
        Objects.requireNonNull(action, Point.nullMessage("withEventAtFraction", "the event action"));
        return withEvent(PathEvent.atFraction(fraction, action));
    }

    /** @return a copy with an event at fraction f of the length */
    public Path withEventAtFraction(double fraction, Runnable action) {
        Objects.requireNonNull(action, Point.nullMessage("withEventAtFraction", "the event action"));
        return withEvent(PathEvent.atFraction(fraction, action));
    }

    /** @return a copy with an event at the point index the user wrote */
    public Path withEventAtPoint(int pointIndex, Command action) {
        Objects.requireNonNull(action, Point.nullMessage("withEventAtPoint", "the event action"));
        return withEvent(PathEvent.atPoint(pointIndex, action));
    }

    /** @return a copy with an event at the point index the user wrote */
    public Path withEventAtPoint(int pointIndex, Runnable action) {
        Objects.requireNonNull(action, Point.nullMessage("withEventAtPoint", "the event action"));
        return withEvent(PathEvent.atPoint(pointIndex, action));
    }

    /**
     * Mode NONE returns this. On a flipped copy, the same mode and field size return this; another
     * mode or field size returns that flip of the blue source.
     *
     * @return the flipped path, memoized per mode and field size
     */
    public Path flipped(PathFollowerConfig config) {
        Objects.requireNonNull(config, Point.nullMessage("flipped", "the config"));
        FlipMode mode = Objects.requireNonNull(config.flipMode(), Point.nullMessage("flipped", "the flip mode"));
        if (mode == FlipMode.NONE) {
            return this;
        }
        FlipKey key = new FlipKey(mode, config.fieldLength(), config.fieldWidth());
        if (blueSource != null && key.equals(flipKey)) {
            return this;
        }
        return flippedBy(key);
    }

    /** @return the blue path flipped by key, memoized on the blue path */
    private Path flippedBy(FlipKey key) {
        if (blueSource != null) {
            return blueSource.flippedBy(key);
        }
        synchronized (flipCache) {
            Path cached = flipCache.get(key);
            if (cached != null) {
                return cached;
            }
            FlipMode mode = key.mode();
            double fl = key.fieldLength();
            double fw = key.fieldWidth();
            List<Point> flippedPoints = new ArrayList<>(points.size());
            for (Point p : points) {
                flippedPoints.add(PathFlipper.flipPoint(p, mode, fl, fw));
            }
            Translation2d flippedTarget = faceTarget == null ? null
                    : PathFlipper.flipTranslation(faceTarget, mode, fl, fw);
            Path result = new Path(Collections.unmodifiableList(flippedPoints), PathFlipper.flip(table, mode, fl, fw),
                    constraints, flippedTarget, events, name, warnings, this, key);
            flipCache.put(key, result);
            return result;
        }
    }

    /**
     * Red: {@code flipped(config)} (this if this is already that flip, or for mode NONE). Blue or
     * unknown: the blue path (the blue source of a flipped copy).
     *
     * @return the path for the alliance the DriverStation reports
     */
    public Path forCurrentAlliance(PathFollowerConfig config) {
        Objects.requireNonNull(config, Point.nullMessage("forCurrentAlliance", "the config"));
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return flipped(config);
        }
        return blueSource != null ? blueSource : this;
    }

    /** @return true if this path is a flipped copy */
    public boolean isFlipped() {
        return blueSource != null;
    }

    /** @return the name, default "path" + a counter */
    public String name() {
        return name;
    }

    /** @return length L [m] */
    public double length() {
        return table.length();
    }

    /** @return number of points as given (before merging) */
    public int pointCount() {
        return points.size();
    }

    /** @return the points as given */
    public List<Point> points() {
        return points;
    }

    /** @return s of waypoint i (5.9.4); merged points map to the kept one */
    public double sAtPoint(int pointIndex) {
        return table.waypointS(checkIndex(pointIndex));
    }

    /** @return the per-path constraints, empty = resolved when the command starts */
    public Optional<PathConstraints> constraints() {
        return Optional.ofNullable(constraints);
    }

    /** @return the face target, if set */
    public Optional<Translation2d> faceTarget() {
        return Optional.ofNullable(faceTarget);
    }

    /** @return events resolved to s, in the order added */
    public List<ResolvedEvent> events() {
        return events;
    }

    /** @return every warning reported at build time; the texts keep the automatic name given at construction */
    public List<String> warnings() {
        return warnings;
    }

    /** @return P_0 */
    public Translation2d startPoint() {
        return points.get(0).position();
    }

    /** @return P_{n-1} */
    public Translation2d endPoint() {
        // the kept last point: the first of the trailing points at s = L (later duplicates were merged into it)
        double length = table.length();
        int i = points.size() - 1;
        while (i > 0 && table.waypointS(i - 1) == length) {
            i--;
        }
        return points.get(i).position();
    }

    /** @return P_0's angle if set */
    public Optional<Rotation2d> startHeading() {
        // P_0 and the points merged into it (all at s = 0); the later angle wins (5.8.2)
        Optional<Rotation2d> angle = Optional.empty();
        for (int i = 0; i < points.size() && table.waypointS(i) == 0.0; i++) {
            if (points.get(i).angle().isPresent()) {
                angle = points.get(i).angle();
            }
        }
        return angle;
    }

    /** @return the sample at s clamped to [0, L]; T is the right limit at a STOP */
    public PathSample sampleAt(double s) {
        return table.evaluate(s);
    }

    /** @return poses along the path, rotation = direction of travel */
    public List<Pose2d> polyline(double spacingMeters) {
        if (!(spacingMeters > 0) || !Double.isFinite(spacingMeters)) {
            throw new IllegalArgumentException("polyline(" + spacingMeters
                    + "): the spacing must be a finite number more than 0 (meters), for example polyline(0.05).");
        }
        double length = table.length();
        int count = (int) Math.ceil(length / spacingMeters);
        List<Pose2d> poses = new ArrayList<>(count + 1);
        for (int i = 0; i <= count; i++) {
            PathSample sample = table.evaluate(Math.min(length, i * spacingMeters));
            poses.add(new Pose2d(sample.position(), sample.tangent()));
        }
        return Collections.unmodifiableList(poses);
    }

    /** Internal: the built geometry of this path (flipped if {@link #isFlipped()}). */
    SegmentTable table() {
        return table;
    }

    /** @return the event index i, checked (V19) */
    private int checkIndex(int i) {
        if (i < 0 || i >= points.size()) {
            throw new IllegalArgumentException("Event point index " + i + " does not exist: the path has "
                    + points.size() + " points, numbered 0 to " + (points.size() - 1)
                    + ". Index 0 is the first point you passed to new Path(...).");
        }
        return i;
    }

    /** @return the points as an unmodifiable list; throws V3 for null, V1 for fewer than 2 */
    private static List<Point> checkPoints(String method, Point[] points) {
        Objects.requireNonNull(points, Point.nullMessage(method, "the points array"));
        for (int i = 0; i < points.length; i++) {
            Objects.requireNonNull(points[i], Point.nullMessage(method, "point " + i));
        }
        if (points.length < 2) {
            throw new IllegalArgumentException("Path needs at least 2 points (a start and an end), but you gave "
                    + points.length + ". Example: new Path(new Point(1, 2), new Point(3, 4)).");
        }
        return Collections.unmodifiableList(Arrays.asList(points.clone()));
    }
}
