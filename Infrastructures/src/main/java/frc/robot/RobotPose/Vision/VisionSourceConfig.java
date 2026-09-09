package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Shared construction-parameter object for VisionSource implementations. Bundles a
 * human-readable source name with the device's physical offset from the robot's center,
 * the two pieces of static configuration every source needs regardless of what kind of
 * device it wraps.
 *
 * Confirmed design: this is the general shape the user pointed to (their own old
 * VisionSourceConfig(String sourceName, Transform3d offset) record). A source-specific
 * live dependency that isn't static configuration -- e.g. LimelightTagCamera3d's
 * Supplier<Rotation2d> gyro heading -- stays a separate constructor parameter rather than
 * being folded into this record, since a Supplier is a wiring reference, not a value.
 *
 * @param sourceName The device's identifying name (e.g. a Limelight's NetworkTables name).
 *                    For a Limelight this is its NT name ("" for the default/only camera on
 *                    a single-camera robot); for Quest this is any label the caller finds
 *                    useful (Quest itself doesn't use a name lookup the way Limelight does).
 * @param offset      The device's physical offset from the robot's center. 
 */
public record VisionSourceConfig(String sourceName, Transform3d offset) {
}