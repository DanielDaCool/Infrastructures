package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Path.PathFollowerConfig.FlipMode;

/**
 * Points and constraints of one path, with the built geometry (blue frame, or the flipped frame
 * for a flipped copy) (design doc section 9.4). Immutable: each {@code withX} returns a new Path
 * that shares the geometry. Build warnings are kept in {@link #warnings()} and printed once to the
 * Driver Station when the path is built.
 */
public final class Path {

    private static final AtomicInteger COUNTER = new AtomicInteger();

    private final List<Point> points;
    private final PathGeometry geometry;
    private final PathConstraints constraints;   // null = resolved when the command starts
    private final String name;
    private final List<String> warnings;

    /**
     * Builds the geometry. Throws on bad input (V1, V3, V6, V30), warns for repairs.
     *
     * @param points two or more points
     */
    public Path(Point... points) {
        this(null, checkPoints(points));
    }

    /**
     * @param constraints per-path constraints
     * @param points      two or more points
     */
    public Path(PathConstraints constraints, Point... points) {
        this(Objects.requireNonNull(constraints, Point.nullMessage("new Path", "the constraints", "a PathConstraints (for example PathConstraints.fromConfig(config))")),
                checkPoints(points));
    }

    /** Builds from a checked list and reports the build warnings once. */
    private Path(PathConstraints constraints, List<Point> points) {
        this.points = points;
        this.name = "path" + COUNTER.incrementAndGet();
        List<String> w = new ArrayList<>();
        this.geometry = PathGeometry.build(points, name, w);
        this.warnings = Collections.unmodifiableList(w);
        this.constraints = constraints;
        for (String text : warnings) {
            DriverStation.reportWarning(text, false);
        }
    }

    /** Copy constructor: shares the geometry. */
    private Path(List<Point> points, PathGeometry geometry, PathConstraints constraints, String name,
                 List<String> warnings) {
        this.points = points;
        this.geometry = geometry;
        this.constraints = constraints;
        this.name = name;
        this.warnings = warnings;
    }

    /** @return a path through the points */
    public static Path of(List<Point> points) {
        Objects.requireNonNull(points, Point.nullMessage("Path.of", "the points list", "a list of Points (for example List.of(new Point(1, 2), new Point(3, 4)))"));
        return new Path(points.toArray(new Point[0]));
    }

    /**
     * The name is used in the command name and in run messages; warning texts keep the automatic
     * name the path had when it was built ({@code Path "path3", ...}).
     *
     * @return a copy with a name
     */
    public Path withName(String name) {
        Objects.requireNonNull(name, Point.nullMessage("withName", "the name", "a text (for example withName(\"drive to reef\"))"));
        return new Path(points, geometry, constraints, name, warnings);
    }

    /** @return a copy with per-path constraints */
    public Path withConstraints(PathConstraints constraints) {
        Objects.requireNonNull(constraints, Point.nullMessage("withConstraints", "the constraints", "a PathConstraints (for example PathConstraints.fromConfig(config))"));
        return new Path(points, geometry, constraints, name, warnings);
    }

    /**
     * Mode NONE returns this. Pass the blue path: flipping a flipped path turns it back.
     *
     * @param mode        NONE, MIRROR or ROTATE
     * @param fieldLength field length [m]
     * @param fieldWidth  field width [m]
     * @return the flipped path (a new object on every call)
     */
    public Path flipped(FlipMode mode, double fieldLength, double fieldWidth) {
        Objects.requireNonNull(mode, Point.nullMessage("flipped", "the flip mode", "a FlipMode (for example FlipMode.ROTATE)"));
        if (mode == FlipMode.NONE) {
            return this;
        }
        List<Point> flippedPoints = new ArrayList<>(points.size());
        for (Point p : points) {
            Translation2d pos = PathGeometry.flip(p.position(), mode, fieldLength, fieldWidth);
            Rotation2d angle = p.angle().map(a -> PathGeometry.flip(a, mode)).orElse(null);
            flippedPoints.add(p.withPositionAndAngle(pos.getX(), pos.getY(), angle));
        }
        return new Path(Collections.unmodifiableList(flippedPoints),
                geometry.flipped(mode, fieldLength, fieldWidth), constraints, name, warnings);
    }

    /** @return {@code flipped(config.flipMode(), config.fieldLength(), config.fieldWidth())} */
    public Path flipped(PathFollowerConfig config) {
        Objects.requireNonNull(config, Point.nullMessage("flipped", "the config", "a PathFollowerConfig (for example PathFollowerConfig.defaults())"));
        return flipped(config.flipMode(), config.fieldLength(), config.fieldWidth());
    }

    /** @return the name, default "path" + a counter */
    public String name() {
        return name;
    }

    /** @return length L [m] */
    public double length() {
        return geometry.length();
    }

    /** @return number of points as given (before merging) */
    public int pointCount() {
        return points.size();
    }

    /** @return the points as given */
    public List<Point> points() {
        return points;
    }

    /**
     * @param pointIndex index of the point as given, 0 = first
     * @return s of that point (5.9.4); merged points map to the kept one. Use {@code s / length()}
     *     with the command's {@code progress()} to act at a point.
     */
    public double sAtPoint(int pointIndex) {
        if (pointIndex < 0 || pointIndex >= points.size()) {
            throw new IllegalArgumentException("Point index " + pointIndex + " does not exist: the path has "
                    + points.size() + " points, numbered 0 to " + (points.size() - 1)
                    + ". Index 0 is the first point you passed to new Path(...). Use an index in that range, for example"
                    + " sAtPoint(1).");
        }
        return geometry.waypointS(pointIndex);
    }

    /** @return the per-path constraints, empty = resolved when the command starts */
    public Optional<PathConstraints> constraints() {
        return Optional.ofNullable(constraints);
    }

    /** @return every warning reported at build time; the texts keep the automatic name given at construction */
    public List<String> warnings() {
        return warnings;
    }

    /** @return P_0 */
    public Translation2d startPoint() {
        return points.get(0).position();
    }

    /** @return P_{n-1}: the kept last point (later duplicates at s = L were merged into it) */
    public Translation2d endPoint() {
        double length = geometry.length();
        int i = points.size() - 1;
        while (i > 0 && geometry.waypointS(i - 1) == length) {
            i--;
        }
        return points.get(i).position();
    }

    /** @return P_0's angle if set (P_0 and the points merged into it; the later angle wins, 5.8.2) */
    public Optional<Rotation2d> startHeading() {
        Optional<Rotation2d> angle = Optional.empty();
        for (int i = 0; i < points.size() && geometry.waypointS(i) == 0.0; i++) {
            if (points.get(i).angle().isPresent()) {
                angle = points.get(i).angle();
            }
        }
        return angle;
    }

    /** @return poses along the path, rotation = direction of travel */
    public List<Pose2d> polyline(double spacingMeters) {
        if (!(spacingMeters > 0) || !Double.isFinite(spacingMeters)) {
            throw new IllegalArgumentException("polyline(" + spacingMeters
                    + "): the spacing must be a finite number more than 0 (meters), for example polyline(0.05).");
        }
        double length = geometry.length();
        int count = (int) Math.ceil(length / spacingMeters);
        List<Pose2d> poses = new ArrayList<>(count + 1);
        for (int i = 0; i <= count; i++) {
            PathGeometry.PathSample sample = geometry.evaluate(Math.min(length, i * spacingMeters));
            poses.add(new Pose2d(sample.position(), sample.tangent()));
        }
        return Collections.unmodifiableList(poses);
    }

    /** Internal: the built geometry of this path (flipped for a flipped path). */
    PathGeometry geometry() {
        return geometry;
    }

    /** @return the points as an unmodifiable list; throws V3 for null, V1 for fewer than 2 */
    private static List<Point> checkPoints(Point[] points) {
        Objects.requireNonNull(points, Point.nullMessage("new Path", "the points array", "the points (for example new Path(new Point(1, 2), new Point(3, 4)))"));
        for (int i = 0; i < points.length; i++) {
            Objects.requireNonNull(points[i], Point.nullMessage("new Path", "point " + i, "a Point for every position (for example new Point(1, 2))"));
        }
        if (points.length < 2) {
            throw new IllegalArgumentException("Path needs at least 2 points (a start and an end), but you gave "
                    + points.length + ". Example: new Path(new Point(1, 2), new Point(3, 4)).");
        }
        return Collections.unmodifiableList(Arrays.asList(points.clone()));
    }
}
