package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera2d;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera3d;
import frc.robot.RobotPose.Vision.VisionTypes.Quest;

/**
 * Settings shared by every vision source. Each subclass sets {@link #visionSourceType}, which
 * decides which {@link VisionSource} class {@code VisionConfig.addSource} creates.
 *
 * <p><b>Frame contract:</b> {@link #offset} is always in the WPILib robot frame (x forward,
 * y left, z up, rotations CCW positive), whatever the device. A source whose device expects
 * another frame overrides {@link #sourceFrame()}, and sends {@link #getSourceOffset()} to the
 * device. Only the config classes convert frames; the sources never flip signs by hand.
 *
 * <p><b>Exception:</b> the 2D Limelight ({@link LimelightTagCamera2dConfig}) reads only the
 * offset's height, pitch and yaw, and takes its pitch as positive = camera tilted up, the
 * opposite of WPILib's right-hand pitch (where positive points the camera down).
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
     * (meters, radians), in the WPILib robot frame (x forward, y left, z up, CCW positive).
     */
    public Transform3d offset;
    /**
     * Base measurement std devs (x meters, y meters, theta radians). Not used by the Limelights
     * (their std dev comes from the tag distance and count); for the Quest the x value is the
     * smallest std dev of its thinned frames (see {@link QuestConfig}).
     */
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

    /**
     * The device's robot frame, as a matrix C that maps a WPILib robot-frame vector to the
     * device's frame. Identity (the device uses the WPILib frame) unless overridden.
     */
    protected Matrix<N3, N3> sourceFrame() {
        return Matrix.eye(Nat.N3());
    }

    /**
     * {@link #offset} in the device's frame ({@link #sourceFrame()}): the position is
     * {@code p' = C * p} and the rotation {@code R' = C * R * C^T}. This is what the device
     * should be told.
     */
    public final Transform3d getSourceOffset() {
        Matrix<N3, N3> c = sourceFrame();
        Matrix<N3, N1> position = c.times(offset.getTranslation().toVector());
        Matrix<N3, N3> rotation = c.times(offset.getRotation().toMatrix()).times(c.transpose());
        return new Transform3d(new Translation3d(position.get(0, 0), position.get(1, 0), position.get(2, 0)),
                new Rotation3d(rotation));
    }
}