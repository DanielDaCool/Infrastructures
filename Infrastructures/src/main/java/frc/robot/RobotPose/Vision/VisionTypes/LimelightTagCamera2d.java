package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotPose.RobotPose;
import frc.robot.RobotPose.Vision.BaseVisionSource;
import frc.robot.RobotPose.Vision.LimelightHelpers;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.visionConfigs.LimelightTagCamera2dConfig;

/**
 * A Limelight used in 2D mode: the robot's position is calculated from the angles to one
 * AprilTag ({@code tx}, {@code ty}, {@code tid}) plus the tag's known place on the field.
 * Only x and y are measured; the heading is taken from the pose estimate.
 *
 * <p>The math, per new frame:
 * <ol>
 * <li>Horizontal distance camera to tag: {@code |tagHeight - cameraHeight| / tan(cameraPitch + ty)},
 * divided by {@code cos(tx)}.</li>
 * <li>Camera to tag vector: that distance at angle {@code cameraYaw - tx} (robot-relative).</li>
 * <li>Robot to tag = robot to camera + camera to tag (robot-relative), rotated to the field
 * by the heading at the frame's capture time.</li>
 * <li>Robot position = tag position - robot to tag.</li>
 * </ol>
 *
 * <p>Reads the Limelight's NetworkTables table {@code "limelight-" + name} directly.
 */
public class LimelightTagCamera2d extends BaseVisionSource {
    private String limelightName;
    private NetworkTable Table;

    /** Capture time of {@link #pose}. */
    private double timestampSeconds;
    /** Whether this loop's periodic() produced a new, finite pose. */
    private boolean hasNewPose;
    /** Heartbeat of the last frame processed; it increments once per camera frame. */
    private double lastHeartbeat = Double.NaN;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    /** Id of the tag used for the last pose (-1 until one is used). */
    private int lastTagId = -1;
    /** Floor distance camera to tag for the last pose (meters). */
    private double lastTagDistance;

    private double lastFrameCounterValue = 0;
    private double lastFrameCounterChangeTime = -1;

    /** Disconnected if the heartbeat hasn't changed for this long. */
    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 0.5;

    /**
     * @param config The camera's config. The table read is {@code "limelight-" + config.name}.
     */
    public LimelightTagCamera2d(LimelightTagCamera2dConfig config) {
        super(config);
        limelightName = "limelight-" + config.name;
        Table = NetworkTableInstance.getDefault().getTable(limelightName);
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    /**
     * @return Whether the camera sees a tag ({@code tv}). Does not check if the frame is new;
     *         {@link #getPoseEstimates()} handles that.
     */
    @Override
    public boolean shouldUpdate() {
        if (Table == null) {
            return false;
        }
        return Table.getEntry("tv").getDouble(0.0) >= 0.1;
    }

    /**
     * @return This loop's measurement, or an empty list if there is no new frame. The heading
     *         std dev is infinite because the heading was copied from the estimate.
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
     * @return This loop's measurement, or an empty list if there is no new frame. The heading
     *         std dev is infinite because the heading was copied from the estimate.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        if (!hasNewPose) {
            return List.of();
        }

        return List.of(new TimestampedVisionMeasurement(pose, timestampSeconds, 
            VecBuilder.fill(std.get(0, 0), std.get(1, 0), Double.POSITIVE_INFINITY)));
    }

   
    /**
     * Calculates a new pose if the camera sees a tag, the frame is new (heartbeat changed),
     * and the tag is in the field layout. Otherwise there is no measurement this loop.
     */
    @Override
    public void periodic() {
        super.periodic();

        hasNewPose = false;

        if (!shouldUpdate()) {
            return;
        }

        // Until the camera publishes a newer frame, the same tx/ty are read back every loop.
        double heartbeat = LimelightHelpers.getHeartbeat(limelightName);
        if (heartbeat == lastHeartbeat) {
            return;
        }
        lastHeartbeat = heartbeat;

        // A tag that isn't in the field layout has no known position to measure from.
        if (aprilTagFieldLayout.getTagPose((int) Table.getEntry("tid").getDouble(0.0)).isEmpty()) {
            return;
        }

        updatePose();
        // e.g. a tag at the camera's height makes the range dh / tan(0)
        hasNewPose = Double.isFinite(pose.getX()) && Double.isFinite(pose.getY());
    }

    /**
     * Sets {@link #timestampSeconds} (now minus pipeline + capture latency) and calculates
     * {@link #pose} from the current tag.
     */
    private Pose2d updatePose() {
        double latency = (Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0))/1000.0;
        double timestampSeconds = Timer.getFPGATimestamp() - latency;

        Rotation2d heading = RobotPose.getInstance().getEstimatedPoseAt(timestampSeconds).getRotation();
        lastTagId = (int) Table.getEntry("tid").getDouble(-1);
        lastTagDistance = getDistanceFromCamera();
        pose = new Pose2d((getTag().toTranslation2d()).minus(getRobotToTag(heading)), RobotPose.getInstance().getGyroAngle());
        return pose;
    }

    /**
     * Vector from the robot center to the tag, on the field.
     *
     * <p>Robot to camera (the offset) and camera to tag are both relative to the robot, so
     * they are added first and the sum is rotated to the field by the heading. (Before issue
     * #13 only the offset was rotated, so the pose was only right at heading 0 and was meters
     * off at 90 and 180 degrees.)
     *
     * @param heading The robot's field heading when the frame was captured.
     */
    private Translation2d getRobotToTag(Rotation2d heading) {
        return getCameraToTag().plus(
            offset.getTranslation().toTranslation2d()).rotateBy(heading);
    }

    /**
     * Vector from the camera to the tag, relative to the robot (not the field): the floor
     * distance at angle {@code cameraYaw - tx} (tx is positive to the right).
     */
    private Translation2d getCameraToTag() {
        return new Translation2d(getDistanceFromCamera(),
            new Rotation2d(Math.toRadians(-Table.getEntry("tx").getDouble(0.0)) + offset.getRotation().getZ()));
    }

    /**
     * Distance from the camera to the tag on the floor plane:
     * {@code |tagZ - cameraZ| / tan(cameraPitch + ty)} is the distance straight ahead of the
     * camera, and dividing by {@code cos(tx)} adds the sideways part. Pitch is positive = up.
     * Not finite when {@code cameraPitch + ty} is 0 (tag at the camera's height).
     */
    private double getDistanceFromCamera() {
        double deltaHeight = getTag().getZ() - offset.getZ();
        double alpha = offset.getRotation().getY() + Math.toRadians(Table.getEntry("ty").getDouble(0.0));
        double distance = Math.abs(deltaHeight / Math.tan(alpha)) / Math.cos(Math.toRadians(Table.getEntry("tx").getDouble(0.0)));

        return distance;   
    }

    /** Field position of the tag the camera sees ({@code tid}), or (0, 0, 0) if unknown. */
    private Translation3d getTag() {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose((int) Table.getEntry("tid").getDouble(0.0));
        if (tagPose.isEmpty()) {
            return new Translation3d();
        }
        return tagPose.get().getTranslation();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> shouldUpdate(), null);
        builder.addIntegerProperty("tag id", () -> lastTagId, null);
        builder.addDoubleProperty("tag distance m", () -> lastTagDistance, null);
    }
}