package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A single, self-contained, self-scored vision pose measurement. Each VisionSource computes
 * and reports its OWN stdDevs per-measurement 
 *
 * @param pose              The measured field-relative robot pose.
 * @param timestampSeconds  When this measurement was actually captured (e.g. FPGA time
 *                          minus the source's own reported latency)
 * @param stdDevs           This specific measurement's own confidence, per-axis
 *                          (x meters, y meters, theta radians)
 */
public record TimestampedVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
}