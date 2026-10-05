package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotPose.RobotPose;
import frc.robot.RobotPose.Vision.BaseVisionSource;
import frc.robot.RobotPose.Vision.LimelightHelpers;
import frc.robot.RobotPose.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;

/**
 * A Limelight using MegaTag2: the Limelight itself calculates the robot pose from every tag it
 * sees, using the robot heading we send it each loop. Only x and y are used; MegaTag2's yaw is
 * just the heading we sent.
 *
 * <p>The camera's position on the robot (the config offset) is sent to the Limelight once, at
 * construction.
 */
public class LimelightTagCamera3d extends BaseVisionSource {
    private String limelightName;

    /** Last MegaTag2 estimate read (blue-alliance origin), or null if none. */
    private PoseEstimate poseEstimate;
    /** Capture time of the last frame reported, so each frame is fused only once. */
    private double lastReportedTimestampSeconds = Double.NaN;
    /** Whether this loop's periodic() read a new frame that has at least one tag. */
    private boolean hasNewPose;

    private double lastFrameCounterValue = 0;
    private double lastFrameCounterChangeTime = -1;

    /** Disconnected if the heartbeat hasn't changed for this long. */
    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 0.5;

    /**
     * Sends the config offset to the Limelight as its camera pose in robot space (meters,
     * degrees). This overwrites what was set in the Limelight web UI.
     *
     * @param config The camera's config. The Limelight name is {@code "limelight-" + config.name}.
     */
    public LimelightTagCamera3d(LimelightTagCamera3dConfig config) {
        super(config);
        limelightName = "limelight-" + config.name;
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, 
            offset.getX(), 
            offset.getY(), 
            offset.getZ(), 
            Math.toDegrees(offset.getRotation().getX()), 
            Math.toDegrees(offset.getRotation().getY()), 
            Math.toDegrees(offset.getRotation().getZ()));
    }

    /**
     * @return Whether the camera sees a tag ({@code tv}). Does not check if the frame is new;
     *         {@link #getPoseEstimates()} handles that.
     */
    @Override
    public boolean shouldUpdate() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * The Limelight's heartbeat counter goes up once per frame while it is running, so the
     * camera counts as connected if the heartbeat changed in the last
     * {@link #CAMERA_STALE_TIMEOUT_SECONDS}. Only updates when called (it's called by the
     * dashboard).
     */
    @Override
    public boolean isConnected() {
        double currentFrameCounter = LimelightHelpers.getHeartbeat(limelightName);
        double now = Timer.getFPGATimestamp();

        if (currentFrameCounter != lastFrameCounterValue) {
            lastFrameCounterValue = currentFrameCounter;
            lastFrameCounterChangeTime = now;
        }

        return (now - lastFrameCounterChangeTime) < CAMERA_STALE_TIMEOUT_SECONDS;
    }

    /**
     * @return This loop's MegaTag2 measurement, or an empty list if there is no new frame with
     *         a tag. The heading std dev is infinite because the heading is the one we sent.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        if (!hasNewPose) {
            return List.of();
        }

        return List.of(new TimestampedVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, 
            VecBuilder.fill(std.get(0, 0), std.get(1, 0), Double.POSITIVE_INFINITY)));
    }

    /**
     * Sends the current estimated heading to the Limelight (MegaTag2 needs it), then reads
     * the latest MegaTag2 estimate. A frame counts as new only if its timestamp differs from
     * the last one used, so each frame is used once.
     *
     * <p>The Limelight uses the heading on its next frame, so the frame read here was made
     * with a heading sent in an earlier loop.
     */
    @Override
    public void periodic() {
        super.periodic();
        
        Rotation2d heading = RobotPose.getInstance().getEstimatedPose().getRotation();
        LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        
        // Until the camera publishes a newer frame, the same estimate is read back every loop.
        hasNewPose = poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.timestampSeconds != lastReportedTimestampSeconds;
        if (hasNewPose) {
            pose = poseEstimate.pose;
            lastReportedTimestampSeconds = poseEstimate.timestampSeconds;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> shouldUpdate(), null);
        builder.addIntegerProperty("tag count", () -> poseEstimate == null ? 0 : poseEstimate.tagCount, null);
        builder.addDoubleProperty("avg tag distance m", () -> poseEstimate == null ? 0 : poseEstimate.avgTagDist, null);
    }
}