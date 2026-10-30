package frc.robot.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Config for a {@link frc.robot.RobotPose.Vision.VisionTypes.Quest}. The offset is robot
 * center to headset. The name is only used for the dashboard.
 *
 * <p>The public fields below have the values tested in simulation; change them after the
 * constructor if the robot measurements say so.
 */
public class QuestConfig extends BaseVisionSourceConfig {
    /**
     * QuestNav stamps a frame with the time it was published, not captured; this much is
     * subtracted (seconds). Measure: drive back and forth and find the shift that lines the
     * Quest position up with the odometry.
     */
    public double latencySeconds = 0.03;
    /**
     * x/y std dev of the frames that are used (meters), unless the config std is bigger. Large
     * on purpose: Quest errors are the same from frame to frame, so it must not outvote the
     * cameras.
     */
    public double thinnedStd = 0.4;
    /** At most one frame per this long is used (seconds). */
    public double periodSeconds = 0.1;
    /** The Quest moving this much differently from the odometry between two frames is a jump (meters). */
    public double jumpMeters = 0.3;
    /** No frame for this long means tracking was lost (seconds). */
    public double gapSeconds = 0.2;
    /** Frames are ignored this long after a jump or lost tracking, then the Quest is re-aligned (seconds). */
    public double blackoutSeconds = 0.5;

    public QuestConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        super(name, offset, std);
        visionSourceType = VisionSourceType.Quest;
    }
}
