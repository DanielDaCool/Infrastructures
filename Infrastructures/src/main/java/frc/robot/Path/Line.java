package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/** Straight segment from start to end [m]. */
public final class Line extends Segment {
    private final Translation2d start;
    private final Translation2d direction;
    private final double length;

    public Line(Translation2d start, Translation2d end) {
        this.start = start;
        this.length = end.minus(start).getNorm();
        if (!(length > 1e-9)) throw new IllegalArgumentException("Line has zero length");
        this.direction = end.minus(start).div(length);
    }

    @Override
    public double length() { return length; }

    @Override
    public Translation2d pointAt(double distance) { return start.plus(direction.times(distance)); }

    @Override
    public Translation2d tangentAt(double distance) { return direction; }

    @Override
    public double project(Translation2d point) { return point.minus(start).dot(direction); }

    @Override
    public double curvature() { return 0; }
}
