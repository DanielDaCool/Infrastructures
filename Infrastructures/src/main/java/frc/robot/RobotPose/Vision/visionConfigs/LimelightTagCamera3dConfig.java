package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Config for a {@link frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera3d}. The
 * offset is sent to the Limelight at startup as its camera pose in robot space.
 *
 * <p>The offset is given in the WPILib robot frame like every other config. The Limelight's
 * robot space has y pointing right, so {@link #sourceFrame()} mirrors y: a camera on the
 * right (y -0.3) is sent as side +0.3, and a yaw of -90 deg (facing right) as +90 deg.
 */
public class LimelightTagCamera3dConfig extends BaseVisionSourceConfig {
    public LimelightTagCamera3dConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        super(name, offset, std);
        visionSourceType = VisionSourceType.Limelight3d;
    }

    /** The Limelight's robot space: x forward, y right, z up, so diag(1, -1, 1). */
    @Override
    protected Matrix<N3, N3> sourceFrame() {
        return MatBuilder.fill(Nat.N3(), Nat.N3(),
                1, 0, 0,
                0, -1, 0,
                0, 0, 1);
    }
}
