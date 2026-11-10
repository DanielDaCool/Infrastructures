package frc.robot.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Immutable waypoint in blue field coordinates [m]. Each {@code withX} returns a new Point. */
public final class Point {
    private final Translation2d position;
    private final Rotation2d angle; // null = none
    private final boolean stop;
    private final double dwell, radius; // radius NaN = automatic (see Path)

    private Point(Translation2d position, Rotation2d angle, boolean stop, double dwell, double radius) {
        this.position = position; this.angle = angle; this.stop = stop; this.dwell = dwell; this.radius = radius;
    }

    /** @param x field x [m] @param y field y [m] */
    public Point(double x, double y) {
        this(new Translation2d(check(x, "x"), check(y, "y")), null, false, 0, Double.NaN);
    }

    /** Robot heading to reach at this point. */
    public Point withAngle(Rotation2d angle) {
        if (angle == null) throw new IllegalArgumentException("Point angle is null");
        return new Point(position, angle, stop, dwell, radius);
    }

    /** Stop here, then continue. */
    public Point stop() { return stop(0); }

    /** Stop here and wait {@code dwellSeconds} [s] before continuing; ignored on the last point. */
    public Point stop(double dwellSeconds) {
        if (!(check(dwellSeconds, "dwell") >= 0)) throw new IllegalArgumentException("Point dwell < 0");
        return new Point(position, angle, true, dwellSeconds, radius);
    }

    /** Corner radius at this point [m], > 0. */
    public Point withRadius(double meters) {
        if (!(check(meters, "radius") > 0)) throw new IllegalArgumentException("Point radius <= 0");
        return new Point(position, angle, stop, dwell, meters);
    }

    public Translation2d position() { return position; }
    /** Heading here, or null. */ public Rotation2d angle() { return angle; }
    public boolean isStop() { return stop; }
    /** Wait at the stop [s]. */ public double dwell() { return dwell; }
    /** Corner radius [m], NaN = automatic. */ public double radius() { return radius; }

    /** This point on the red side: mirrored across x = field length / 2, or rotated 180 deg about the center. */
    Point flipped(boolean mirror) {
        double fieldLength = Field.layout().getFieldLength(), fieldWidth = Field.layout().getFieldWidth();
        double x = fieldLength - position.getX();
        double y = mirror ? position.getY() : fieldWidth - position.getY();
        Rotation2d flippedAngle = angle == null ? null : mirror ? Rotation2d.kPi.minus(angle) : angle.plus(Rotation2d.kPi);
        return new Point(new Translation2d(x, y), flippedAngle, stop, dwell, radius);
    }

    /** The current season field, loaded once on first use; its size is the flip size. */
    static final class Field {
        private static AprilTagFieldLayout layout;

        static synchronized AprilTagFieldLayout layout() {
            if (layout == null) layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            return layout;
        }
    }

    private static double check(double value, String name) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Point " + name + " is not finite");
        return value;
    }
}
