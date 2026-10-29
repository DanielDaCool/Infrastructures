package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
 * <p>A frame is not used if the robot turned faster than {@link #MAX_YAW_RATE_RADIANS_PER_SECOND}
 * in the {@link #YAW_RATE_WINDOW_SECONDS} before it, or if the heading the Limelight used differs
 * from our heading at the frame time by more than {@link #MAX_HEADING_ERROR_RADIANS} (the
 * Limelight solved it with an old heading: after a gyro reset, while spinning, or with a wrong
 * camera yaw). The x/y std dev grows with the tag distance and shrinks with the tag count (see
 * {@link BaseVisionSource#distanceStdDev}); the config std devs are not used.
 *
 * <p>The camera's position on the robot (the config offset, converted to the Limelight's
 * frame) is sent to the Limelight once, at construction.
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
     * Frames taken while the robot turned faster than this (rad/s) are not used: the heading
     * the Limelight used is up to one loop old. Tune by logging the error against the yaw rate.
     */
    public static final double MAX_YAW_RATE_RADIANS_PER_SECOND = 2.0;
    /** The fastest yaw rate is taken over this long before the frame (seconds). */
    public static final double YAW_RATE_WINDOW_SECONDS = 0.1;
    /** Frames whose heading differs from ours at the frame time by more than this are not used. */
    public static final double MAX_HEADING_ERROR_RADIANS = Math.toRadians(3);
    /** Frames rejected by the yaw rate or heading check since startup (for the dashboard). */
    private int rejectedFrames;
    private double[] sourcePoseSent; // no initializer: set in the constructor, read by the dashboard

    /**
     * Sends the config offset to the Limelight as its camera pose in robot space (meters,
     * degrees). This overwrites what was set in the Limelight web UI. The offset is converted to
     * the Limelight's frame by {@link LimelightTagCamera3dConfig#getSourceOffset()}, and the six
     * values sent are put on {@code vision/<name>/source pose sent} (forward, side, up, roll,
     * pitch, yaw).
     *
     * @param config The camera's config. The Limelight name is {@code "limelight-" + config.name}.
     */
    public LimelightTagCamera3d(LimelightTagCamera3dConfig config) {
        super(config);
        limelightName = "limelight-" + config.name;
        Transform3d sourceOffset = config.getSourceOffset();
        double[] sourcePose = {
            sourceOffset.getX(),
            sourceOffset.getY(),
            sourceOffset.getZ(),
            Math.toDegrees(sourceOffset.getRotation().getX()),
            Math.toDegrees(sourceOffset.getRotation().getY()),
            Math.toDegrees(sourceOffset.getRotation().getZ()) };
        LimelightHelpers.setCameraPose_RobotSpace(limelightName,
            sourcePose[0], sourcePose[1], sourcePose[2], sourcePose[3], sourcePose[4], sourcePose[5]);
        sourcePoseSent = sourcePose;
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
        // MegaTag2's yaw is the heading we sent with SetRobotOrientation, not a measurement.
        double xyStd = distanceStdDev(poseEstimate.avgTagDist, poseEstimate.tagCount);
        return List.of(new TimestampedVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
                VecBuilder.fill(xyStd, xyStd, Double.POSITIVE_INFINITY)));
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

        // MegaTag2 needs the field heading; after a pose reset that is no longer the raw gyro.
        Rotation2d heading = RobotPose.getInstance().getEstimatedPose().getRotation();
        LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Until the camera publishes a newer frame, the same estimate is read back every loop.
        hasNewPose = poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.timestampSeconds != lastReportedTimestampSeconds;
        if (hasNewPose) {
            pose = poseEstimate.pose;
            lastReportedTimestampSeconds = poseEstimate.timestampSeconds;
            RobotPose robotPose = RobotPose.getInstance();
            if (!isUsable(pose.getRotation(),
                    robotPose.getEstimatedPoseAt(poseEstimate.timestampSeconds).getRotation(),
                    robotPose.getPeakYawRate(poseEstimate.timestampSeconds, YAW_RATE_WINDOW_SECONDS))) {
                hasNewPose = false;
                rejectedFrames++;
            }
        }
    }

    /**
     * @param megaTagHeading     The heading in the MegaTag2 pose (the one the Limelight used).
     * @param headingAtFrame     Our estimated heading at the frame time.
     * @param peakYawRate        The fastest yaw rate before the frame (rad/s).
     * @return Whether a MegaTag2 frame can be used (see the class docs).
     */
    static boolean isUsable(Rotation2d megaTagHeading, Rotation2d headingAtFrame, double peakYawRate) {
        return peakYawRate <= MAX_YAW_RATE_RADIANS_PER_SECOND
                && Math.abs(megaTagHeading.minus(headingAtFrame).getRadians()) <= MAX_HEADING_ERROR_RADIANS;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> shouldUpdate(), null);
        builder.addIntegerProperty("tag count", () -> poseEstimate == null ? 0 : poseEstimate.tagCount, null);
        builder.addDoubleProperty("avg tag distance m", () -> poseEstimate == null ? 0 : poseEstimate.avgTagDist, null);
        builder.addIntegerProperty("frames rejected (spin or heading)", () -> rejectedFrames, null);
        // forward, side, up m; roll, pitch, yaw deg, in the Limelight's frame (set before the constructor ends)
        builder.addDoubleArrayProperty("source pose sent", () -> sourcePoseSent == null ? new double[0] : sourcePoseSent, null);
    }
}