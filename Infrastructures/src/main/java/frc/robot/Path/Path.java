package frc.robot.Path;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Points, constraints, face target and events of one path. Builds and holds the geometry
 * (blue frame) and memoizes the flipped copy (design doc section 9.4). Immutable: each
 * {@code withX} returns a new Path that shares the geometry.
 */
public final class Path {

    /**
     * Builds the geometry. Throws on bad input (V1, V3, V6, V30), warns for repairs.
     *
     * @param points two or more points
     */
    public Path(Point... points) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /**
     * @param constraints per-path constraints
     * @param points      two or more points
     */
    public Path(PathConstraints constraints, Point... points) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a path through the points */
    public static Path of(List<Point> points) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a path through the points with per-path constraints */
    public static Path of(PathConstraints constraints, List<Point> points) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with a name (telemetry key {@code Path/<name>}) */
    public Path withName(String name) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with per-path constraints */
    public Path withConstraints(PathConstraints constraints) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy whose robot front faces the target (blue frame) */
    public Path withFaceTarget(Translation2d target) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy without a face target */
    public Path withoutFaceTarget() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with the event, resolved to s now (5.9.5) */
    public Path withEvent(PathEvent event) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with an event at fraction f of the length */
    public Path withEventAtFraction(double fraction, Command action) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with an event at fraction f of the length */
    public Path withEventAtFraction(double fraction, Runnable action) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with an event at the point index the user wrote */
    public Path withEventAtPoint(int pointIndex, Command action) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a copy with an event at the point index the user wrote */
    public Path withEventAtPoint(int pointIndex, Runnable action) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the flipped path, memoized per mode and field size; mode NONE returns this */
    public Path flipped(PathFollowerConfig config) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return this on blue or unknown alliance, {@code flipped(config)} on red */
    public Path forCurrentAlliance(PathFollowerConfig config) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return true if this path is a flipped copy */
    public boolean isFlipped() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the name, default "path" + a counter */
    public String name() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return length L [m] */
    public double length() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return number of points as given (before merging) */
    public int pointCount() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the points as given */
    public List<Point> points() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return s of waypoint i (5.9.4); merged points map to the kept one */
    public double sAtPoint(int pointIndex) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the per-path constraints, empty = resolved when the command starts */
    public Optional<PathConstraints> constraints() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the face target, if set */
    public Optional<Translation2d> faceTarget() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return events resolved to s, in the order added */
    public List<ResolvedEvent> events() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return every warning reported at build time */
    public List<String> warnings() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return P_0 */
    public Translation2d startPoint() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return P_{n-1} */
    public Translation2d endPoint() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return P_0's angle if set */
    public Optional<Rotation2d> startHeading() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the sample at s clamped to [0, L]; T is the right limit at a STOP */
    public PathSample sampleAt(double s) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return poses along the path, rotation = direction of travel */
    public List<Pose2d> polyline(double spacingMeters) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** Internal: the built geometry of this path (flipped if {@link #isFlipped()}). */
    SegmentTable table() {
        throw new UnsupportedOperationException("TODO phase 1");
    }
}
