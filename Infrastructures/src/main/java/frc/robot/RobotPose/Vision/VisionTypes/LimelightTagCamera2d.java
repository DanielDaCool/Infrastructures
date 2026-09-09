package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotPose.Vision.LimelightHelpers;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionSourceConfig;

/**
 * Wraps a single Limelight camera in 2D mode as a VisionSource.
 *
 * 
 *
 */
public class LimelightTagCamera2d implements VisionSource {

    private final String sourceName;
    private Translation3d cameraOffset;

    // --- Connectivity tracking (see isConnected() below) --------------------------------
    private double lastFrameCounterValue = -1.0;
    private double lastFrameCounterChangeTime = 0.0;

    /**
     * How long the Limelight's frame counter (LimelightHelpers.getHeartbeat()) may go
     * without incrementing before this camera is considered disconnected. 
     */
    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 1.0;

    /**
     * @param config Static configuration for this source. sourceName is the Limelight's
     *               NetworkTables name (empty string "" for the default/only Limelight on a
     *               robot with just one camera).
     * */
    public LimelightTagCamera2d(VisionSourceConfig config) {
        this.sourceName = config.sourceName();
        this.cameraOffset = config.offset().getTranslation();
    }

    /**
     * @return if the camera should send it's Pose estimation to RobotPose.
     */
    @Override
    public boolean shouldUpdate() {
        return false;
    }

    /**
     * 
     *
     * @return A list of pose measurements from this loop, or an empty list if none should
     *         be reported (e.g. no valid tag, or the candidate was rejected by your own
     *         confidence logic).
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        throw new UnsupportedOperationException(
                "getPoseEstimates() is not yet implemented -- fill in the pose-estimation math here");
    }

    /**
     * Limelight cameras communicate over NetworkTables; LimelightHelpers does not expose a
     * direct "is this Limelight physically connected" boolean, so connectivity is inferred
     * from LimelightHelpers.getHeartbeat(sourceName) - a counter that increments once per
     * frame while the camera is alive . Since it's a raw counter, not a
     * boolean, connectivity has to be inferred by checking whether it's still CHANGING over
     * time, not just reading it once - so this tracks the last-seen value and when it last
     * changed, and reports disconnected only if the counter has been stuck for longer than
     * CAMERA_STALE_TIMEOUT_SECONDS.
     */
    @Override
    public boolean isConnected() {
        double currentFrameCounter = LimelightHelpers.getHeartbeat(sourceName);
        double now = Timer.getFPGATimestamp();

        if (currentFrameCounter != lastFrameCounterValue) {
            lastFrameCounterValue = currentFrameCounter;
            lastFrameCounterChangeTime = now;
        }

        return (now - lastFrameCounterChangeTime) < CAMERA_STALE_TIMEOUT_SECONDS;
    }

   
    @Override
    public void periodic() {
    }
}