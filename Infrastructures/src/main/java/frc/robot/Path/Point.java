package frc.robot.Path;

import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * One waypoint: position plus optional angle, max speed, corner radius, stop with dwell (design
 * doc section 9.3). Immutable; each {@code withX} returns a new Point and the last call of a
 * kind wins. All coordinates are in the blue frame.
 */
public final class Point {

    private final double x;
    private final double y;
    private final Rotation2d angle;      // null = none
    private final double maxVelocity;    // NaN = none; never 0
    private final double radius;         // NaN = none
    private final boolean stop;
    private final double dwell;

    private Point(double x, double y, Rotation2d angle, double maxVelocity, double radius,
                  boolean stop, double dwell) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.maxVelocity = maxVelocity;
        this.radius = radius;
        this.stop = stop;
        this.dwell = dwell;
    }

    /** @param x blue-frame x [m] @param y blue-frame y [m] */
    public Point(double x, double y) {
        this(checkFinite("Point", x, "x", "meters", x), checkFinite("Point", y, "y", "meters", y), null,
                Double.NaN, Double.NaN, false, 0.0);
    }

    /** @param position blue-frame position */
    public Point(Translation2d position) {
        this(Objects.requireNonNull(position, nullMessage("Point", "the position")).getX(), position.getY());
    }

    /** @param pose blue-frame position plus {@code withAngle(pose.getRotation())} */
    public Point(Pose2d pose) {
        this(checkFinite("Point", Objects.requireNonNull(pose, nullMessage("Point", "the pose")).getX(), "x",
                "meters", pose.getX()),
                checkFinite("Point", pose.getY(), "y", "meters", pose.getY()),
                checkAngle("Point", pose.getRotation()), Double.NaN, Double.NaN, false, 0.0);
    }

    /** @return a copy with a heading anchor (section 6) */
    public Point withAngle(Rotation2d angle) {
        Objects.requireNonNull(angle, nullMessage("withAngle", "the angle"));
        checkAngle("withAngle", angle);
        return new Point(x, y, angle, maxVelocity, radius, stop, dwell);
    }

    /** @return a copy with a max speed here; 0 means {@link #stop()}; v &lt; 0 throws (V11) */
    public Point withVelocity(double metersPerSecond) {
        checkFinite("withVelocity", metersPerSecond, "the speed", "m/s", metersPerSecond);
        if (metersPerSecond < 0) {
            throw new IllegalArgumentException("withVelocity(" + metersPerSecond
                    + "): the speed must be 0 or more (m/s). withVelocity(0) is the same as stop(). "
                    + "To limit the speed at a point use a positive number, for example withVelocity(1.0).");
        }
        if (metersPerSecond == 0) {
            return stop();
        }
        return new Point(x, y, angle, metersPerSecond, radius, false, 0.0);
    }

    /** @return a copy with a corner radius override; 0 = stop corner; r &lt; 0 throws (V9) */
    public Point withRadius(double meters) {
        checkFinite("withRadius", meters, "the radius", "meters", meters);
        if (meters < 0) {
            throw new IllegalArgumentException("withRadius(" + meters
                    + "): the radius must be 0 or more (meters). Use withRadius(0) to make the robot stop "
                    + "at this point, or leave withRadius out so the library chooses the radius.");
        }
        return new Point(x, y, angle, maxVelocity, meters, stop, dwell);
    }

    /**
     * Stops the robot exactly on this point. A stop point is a sharp corner: the robot arrives on
     * a straight line, stops on the point, then continues on a straight line. It has no arc, so it
     * cannot be combined with withRadius(r) for r greater than 0.
     *
     * @return a copy with v = 0 here and no wait
     */
    public Point stop() {
        return stop(0.0);
    }

    /** @return a copy with v = 0 here, then a wait of dwellSeconds; d &lt; 0 throws (V13) */
    public Point stop(double dwellSeconds) {
        checkFinite("stop", dwellSeconds, "the wait time", "seconds", dwellSeconds);
        if (dwellSeconds < 0) {
            throw new IllegalArgumentException("stop(" + dwellSeconds
                    + "): the wait time must be 0 or more (seconds). Use stop() to stop without waiting, "
                    + "or stop(1.5) to wait 1.5 seconds.");
        }
        return new Point(x, y, angle, Double.NaN, radius, true, dwellSeconds);
    }

    /**
     * Internal: a copy at another position with another angle (null = none); speed, radius and
     * stop are kept. Used by the alliance flip.
     */
    Point withPositionAndAngle(double newX, double newY, Rotation2d newAngle) {
        return new Point(newX, newY, newAngle, maxVelocity, radius, stop, dwell);
    }

    /** @return the V3 message for a null argument */
    static String nullMessage(String method, String argument) {
        return method + ": " + argument + " is null. Every argument must be set; see the example in Path.md \"Quick start\".";
    }

    /** @return the V4 message for a NaN or infinite number */
    static String notFiniteMessage(String method, double value, String name, String unit) {
        return method + "(" + value + "): " + name + " must be a finite number (" + unit
                + "), but it is NaN or infinite. A NaN usually comes from a division by zero or from a value "
                + "that was never set.";
    }

    /** @return value; throws V4 if it is NaN or infinite */
    private static double checkFinite(String method, double value, String name, String unit, double shown) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(notFiniteMessage(method, shown, name, unit));
        }
        return value;
    }

    /** @return angle; throws V4 if it is NaN or infinite */
    private static Rotation2d checkAngle(String method, Rotation2d angle) {
        if (!Double.isFinite(angle.getCos()) || !Double.isFinite(angle.getSin())) {
            throw new IllegalArgumentException(notFiniteMessage(method, angle.getRadians(), "the angle", "radians"));
        }
        return angle;
    }

    /** @return the position, blue frame */
    public Translation2d position() {
        return new Translation2d(x, y);
    }

    /** @return x [m] */
    public double x() {
        return x;
    }

    /** @return y [m] */
    public double y() {
        return y;
    }

    /** @return the heading anchor, if set */
    public Optional<Rotation2d> angle() {
        return Optional.ofNullable(angle);
    }

    /** @return the max speed here, empty if not set; never 0 (0 is stored as a stop) */
    public OptionalDouble maxVelocity() {
        return Double.isNaN(maxVelocity) ? OptionalDouble.empty() : OptionalDouble.of(maxVelocity);
    }

    /** @return the corner radius override, if set */
    public OptionalDouble radius() {
        return Double.isNaN(radius) ? OptionalDouble.empty() : OptionalDouble.of(radius);
    }

    /** @return true for stop() / stop(d) / withVelocity(0) */
    public boolean isStop() {
        return stop;
    }

    /** @return 0 unless stop(d) with d &gt; 0 */
    public double dwellSeconds() {
        return dwell;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Point p)) {
            return false;
        }
        return Double.compare(x, p.x) == 0 && Double.compare(y, p.y) == 0
                && Objects.equals(angle, p.angle)
                && Double.compare(maxVelocity, p.maxVelocity) == 0
                && Double.compare(radius, p.radius) == 0
                && stop == p.stop && Double.compare(dwell, p.dwell) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, angle, maxVelocity, radius, stop, dwell);
    }

    /** @return e.g. {@code "Point(3.0, 4.0, angle=90.0deg, stop 1.5 s)"} */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("Point(").append(x).append(", ").append(y);
        if (angle != null) {
            sb.append(", angle=").append(angle.getDegrees()).append("deg");
        }
        if (!Double.isNaN(maxVelocity)) {
            sb.append(", v<=").append(maxVelocity);
        }
        if (!Double.isNaN(radius)) {
            sb.append(", r=").append(radius);
        }
        if (stop) {
            sb.append(dwell > 0 ? ", stop " + dwell + " s" : ", stop");
        }
        return sb.append(')').toString();
    }
}
