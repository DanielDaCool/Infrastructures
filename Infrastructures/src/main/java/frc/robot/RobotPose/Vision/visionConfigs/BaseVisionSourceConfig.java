package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera2d;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera3d;
import frc.robot.RobotPose.Vision.VisionTypes.Quest;

/**
 * Settings shared by every vision source. Each subclass sets {@link #visionSourceType}, which
 * decides which {@link VisionSource} class {@code VisionConfig.addSource} creates.
 */
public abstract class BaseVisionSourceConfig {
    /** Every kind of vision source, and how to create it from its config. */
    public static enum VisionSourceType {
        Limelight2d {
            @Override
            public VisionSource create(BaseVisionSourceConfig config) {
                return new LimelightTagCamera2d((LimelightTagCamera2dConfig) config);
            }
        },
        Limelight3d {
            @Override
            public VisionSource create(BaseVisionSourceConfig config) {
                return new LimelightTagCamera3d((LimelightTagCamera3dConfig) config);
            }
        },
        Quest {
            @Override
            public VisionSource create(BaseVisionSourceConfig config) {
                return new Quest((QuestConfig) config);
            }
        };

        public abstract VisionSource create(BaseVisionSourceConfig config);
    };

    /**
     * Source name. For Limelights the NetworkTables table is {@code "limelight-" + name}, so
     * it has to match the camera's hostname.
     */
    public String name;
    /**
     * Robot center to device: where the camera/headset is mounted and how it is rotated
     * (meters, radians).
     */
    public Transform3d offset;
    /** Base measurement std devs (x meters, y meters, theta radians). */
    public Matrix<N3, N1> std;
    public VisionSourceType visionSourceType = VisionSourceType.Limelight2d;

    /**
     * @param name   Source name (see {@link #name}).
     * @param offset Robot center to device (see {@link #offset}).
     * @param std    Base measurement std devs (x m, y m, theta rad).
     */
    public BaseVisionSourceConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        this.name = name;
        this.offset = offset;
        this.std = std;
    }

    public VisionSourceType getVisionSourceType() {
        return visionSourceType;
    }
}