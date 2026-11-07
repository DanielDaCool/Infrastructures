package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/** One piece of a path, parameterized by distance [m] from its start. */
public abstract class Segment {
    /** Length [m]. */
    public abstract double length();

    /** Field position at distance [m]. */
    public abstract Translation2d pointAt(double distance);

    /** Unit travel direction at distance [m]. */
    public abstract Translation2d tangentAt(double distance);

    /** Distance [m] of the closest point to point; unclamped: negative before the start, > length() after the end. */
    public abstract double project(Translation2d point);

    /** Signed curvature [1/m], positive turning left (CCW); 0 for a line. */
    public abstract double curvature();
}
