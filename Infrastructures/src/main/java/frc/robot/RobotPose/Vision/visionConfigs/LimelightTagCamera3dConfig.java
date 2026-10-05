package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Config for a {@link frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera3d}. The
 * offset is sent to the Limelight at startup as its camera pose in robot space.
 */
public class LimelightTagCamera3dConfig extends BaseVisionSourceConfig {
    public LimelightTagCamera3dConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        super(name, offset, std);
        visionSourceType = VisionSourceType.Limelight3d;
    }
}
