package frc.robot.Path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Read-only geometry sample of a path at arc length s (design doc section 9.11).
 *
 * @param s         arc length [m]
 * @param position  point on the path (blue frame, or the flipped frame for a flipped path)
 * @param tangent   direction of travel
 * @param curvature signed curvature [1/m], positive for a left turn
 */
public record PathSample(double s, Translation2d position, Rotation2d tangent, double curvature) {
}
