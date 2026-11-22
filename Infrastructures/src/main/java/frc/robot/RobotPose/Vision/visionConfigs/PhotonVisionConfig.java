package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Config for a {@link frc.robot.RobotPose.Vision.VisionTypes.PhotonVision} camera. PhotonVision
 * uses the WPILib robot frame, so the offset is passed to it as is (no {@code sourceFrame()}
 * override). {@code name} must match the camera name in the PhotonVision web UI.
 */
public class PhotonVisionConfig extends BaseVisionSourceConfig {
    public PhotonVisionConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        super(name, offset, std);
        visionSourceType = VisionSourceType.PhotonVision;
    }
}
