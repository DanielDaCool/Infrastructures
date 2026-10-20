package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Config for a {@link frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera2d}. Only the
 * offset's height, pitch and yaw are read (besides its x and y position), and they are used in
 * the distance math, so they have to be accurate. Pitch is taken as positive = camera tilted
 * up, the opposite of WPILib's right-hand pitch: an exception to the frame contract in
 * {@link BaseVisionSourceConfig}. Roll is ignored.
 */
public class LimelightTagCamera2dConfig extends BaseVisionSourceConfig {
    public LimelightTagCamera2dConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        super(name, offset, std);
        visionSourceType = VisionSourceType.Limelight2d;
    }
}
