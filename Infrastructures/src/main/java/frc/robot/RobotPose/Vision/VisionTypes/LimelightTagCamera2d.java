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
import edu.wpi.first.networktables.TimestampedDoubleArray;
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
 * <p>Each frame is read once, atomically, from the {@code t2d} array (valid, count, tl, cl,
 * tx, ty, ..., tid; see {@link LimelightHelpers#getT2DArray}), so tx, ty, tid and the latency
 * all belong to the same frame. Its capture time is the time NetworkTables received the array
 * minus the pipeline and capture latency.
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
    /** NetworkTables time (microseconds) of the last t2d array processed. */
    private long lastT2dTimestamp = 0;

    /** Index of each value used in the t2d array. */
    private static final int T2D_VALID = 0;
    private static final int T2D_PIPELINE_LATENCY_MS = 2;
    private static final int T2D_CAPTURE_LATENCY_MS = 3;
    private static final int T2D_TX = 4;
    private static final int T2D_TY = 5;
    private static final int T2D_TID = 9;

    /** tx of the frame being processed (degrees, positive = tag to the right). */
    private double tx;
    /** ty of the frame being processed (degrees, positive = tag above the crosshair). */
    private double ty;
    /** Field position of the tag in the frame being processed. */
    private Translation3d tag = new Translation3d();

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
     * @return Whether this loop's {@link #periodic()} read a new frame with a known tag and got
     *         a finite pose from it. Taken from the same t2d array as the measurement, so it can't
     *         disagree with it.
     */
    @Override
    public boolean shouldUpdate() {
        return hasNewPose;
    }

    /** @return Whether the camera sees a tag right now ({@code tv}); for the dashboard. */
    private boolean seesTag() {
        return Table.getEntry("tv").getDouble(0.0) >= 0.1;
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
     * @return This loop's measurement, or an empty list if there is no new frame. The heading
     *         std dev is infinite because the heading was copied from the estimate.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        if (!hasNewPose) {
            return List.of();
        }
        // The heading is taken from the estimate, not measured, so it carries no information.
        return List.of(new TimestampedVisionMeasurement(pose, timestampSeconds,
                VecBuilder.fill(std.get(0, 0), std.get(1, 0), Double.POSITIVE_INFINITY)));
    }

    /**
     * Calculates a new pose if the t2d array is new, has a target, and its tag is in the field
     * layout. Otherwise there is no measurement this loop.
     */
    @Override
    public void periodic() {
        super.periodic();

        hasNewPose = false;

        TimestampedDoubleArray t2d = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "t2d").getAtomic();
        // Until the camera publishes a newer frame, the same array is read back every loop.
        if (t2d.timestamp == lastT2dTimestamp || t2d.value.length <= T2D_TID) {
            return;
        }
        lastT2dTimestamp = t2d.timestamp;

        if (t2d.value[T2D_VALID] < 0.1) {
            return;
        }

        // A tag that isn't in the field layout has no known position to measure from.
        int tagId = (int) t2d.value[T2D_TID];
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return;
        }

        tag = tagPose.get().getTranslation();
        tx = t2d.value[T2D_TX];
        ty = t2d.value[T2D_TY];
        lastTagId = tagId;
        timestampSeconds = getCaptureTimestampSeconds(t2d);
        updatePose();
        // e.g. a tag at the camera's height makes the range dh / tan(0)
        hasNewPose = Double.isFinite(pose.getX()) && Double.isFinite(pose.getY());
    }

    /**
     * Capture time of a t2d frame (FPGA seconds): the time NetworkTables received it minus the
     * pipeline ({@code tl}) and capture ({@code cl}) latency, both in milliseconds.
     *
     * @param t2d The t2d array with its NetworkTables timestamp (microseconds).
     */
    public static double getCaptureTimestampSeconds(TimestampedDoubleArray t2d) {
        return t2d.timestamp / 1e6
                - (t2d.value[T2D_PIPELINE_LATENCY_MS] + t2d.value[T2D_CAPTURE_LATENCY_MS]) / 1000.0;
    }

    /** Calculates {@link #pose} from the frame read in {@link #periodic()}. */
    private Pose2d updatePose() {
        // Heading at the frame's capture time, not now (the robot may turn during the latency).
        Rotation2d heading = RobotPose.getInstance().getEstimatedPoseAt(timestampSeconds).getRotation();
        lastTagDistance = getDistanceFromCamera();
        pose = new Pose2d(tag.toTranslation2d().minus(getRobotToTag(heading)), heading);
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
            new Rotation2d(Math.toRadians(-tx) + offset.getRotation().getZ()));
    }

    /**
     * Distance from the camera to the tag on the floor plane:
     * {@code |tagZ - cameraZ| / tan(cameraPitch + ty)} is the distance straight ahead of the
     * camera, and dividing by {@code cos(tx)} adds the sideways part. Pitch is positive = up.
     * Not finite when {@code cameraPitch + ty} is 0 (tag at the camera's height).
     */
    private double getDistanceFromCamera() {
        double deltaHeight = tag.getZ() - offset.getZ();
        double alpha = offset.getRotation().getY() + Math.toRadians(ty);
        double distance = Math.abs(deltaHeight / Math.tan(alpha)) / Math.cos(Math.toRadians(tx));

        return distance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> seesTag(), null);
        builder.addIntegerProperty("tag id", () -> lastTagId, null);
        builder.addDoubleProperty("tag distance m", () -> lastTagDistance, null);
    }
}