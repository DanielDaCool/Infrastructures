package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * One vision measurement, as a {@link VisionSource} hands it to RobotPose.
 *
 * @param pose              The measured field pose of the robot center (blue-alliance
 *                          origin).
 * @param timestampSeconds  FPGA time the frame was captured, not when it arrived (e.g. now
 *                          minus the camera's latency).
 * @param stdDevs           How much to trust this measurement, per axis (x meters,
 *                          y meters, theta radians). Use {@code Double.POSITIVE_INFINITY}
 *                          for an axis that wasn't actually measured, e.g. a heading that
 *                          was copied from the estimate.
 */
public record TimestampedVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
}