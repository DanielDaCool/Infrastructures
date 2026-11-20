package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PhotonVisionConfig extends BaseVisionSourceConfig {
    public PhotonVisionConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        super(name, offset, std);
        visionSourceType = VisionSourceType.PhotonVision;
    }

    @Override
    protected Matrix<N3, N3> sourceFrame() {
        return MatBuilder.fill(Nat.N3(), Nat.N3(),
                1, 0, 0,
                0, -1, 0,
                0, 0, 1);
    }
    
}
