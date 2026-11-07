package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/** One piece of a path, parameterized by distance t [m] from its start. */
public abstract class Segment {
    /** Length [m]. */
    public abstract double length();

    /** Field position at distance t [m]. */
    public abstract Translation2d pointAt(double t);

    /** Unit travel direction at distance t [m]. */
    public abstract Translation2d tangentAt(double t);

    /** Distance [m] of the closest point to p; unclamped: negative before the start, > length() after the end. */
    public abstract double project(Translation2d p);

    /** Signed curvature [1/m], positive turning left (CCW); 0 for a line. */
    public abstract double curvature();
}
