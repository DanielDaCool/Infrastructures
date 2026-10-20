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
 * <p>The math, per new frame ({@code p} = camera pitch, positive = tilted up):
 * <ol>
 * <li>Ray to the tag in camera axes: {@code (1, -tan(tx), tan(ty))}.</li>
 * <li>Tilted up by {@code p}: {@code x' = cos p - tan(ty) sin p}, {@code y' = -tan(tx)},
 * {@code z' = sin p + tan(ty) cos p}.</li>
 * <li>Scaled to the tag's height: {@code s = (tagHeight - cameraHeight) / z'}. If {@code s}
 * isn't positive the ray never reaches the tag's height and the frame is dropped.</li>
 * <li>Camera to tag (robot-relative) = {@code (s x', s y')} rotated by the camera yaw.</li>
 * <li>Robot to tag = robot to camera + camera to tag (robot-relative), rotated to the field
 * by the heading at the frame's capture time.</li>
 * <li>Robot position = tag position - robot to tag.</li>
 * </ol>
 *
 * <p>This is exact for a pinhole camera: Limelight's tx and ty are each the atan of the
 * normalized image coordinate on their own axis. Camera roll is not modeled.
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
        // The dashboard reads this from BaseVisionSource's constructor, before Table is set.
        return Table != null && Table.getEntry("tv").getDouble(0.0) >= 0.1;
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
        // e.g. a ray that never reaches the tag's height gives a NaN camera to tag
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
     * Vector from the camera to the tag, relative to the robot (not the field) and level with
     * the floor: the ray {@code (1, -tan(tx), tan(ty))} tilted up by the camera pitch
     * (positive = up), scaled until it reaches the tag's height, and turned by the camera yaw.
     * NaN when the ray never reaches the tag's height (e.g. the tag is at the camera's height,
     * or above it while the ray points down), so the frame is dropped.
     */
    private Translation2d getCameraToTag() {
        double pitch = offset.getRotation().getY();
        double tanTx = Math.tan(Math.toRadians(tx));
        double tanTy = Math.tan(Math.toRadians(ty));

        double forward = Math.cos(pitch) - tanTy * Math.sin(pitch);
        double left = -tanTx;
        double up = Math.sin(pitch) + tanTy * Math.cos(pitch);

        double scale = (tag.getZ() - offset.getZ()) / up;
        if (!(scale > 0)) {
            return new Translation2d(Double.NaN, Double.NaN);
        }
        return new Translation2d(scale * forward, scale * left)
            .rotateBy(new Rotation2d(offset.getRotation().getZ()));
    }

    /** Distance from the camera to the tag on the floor plane (for the dashboard). */
    private double getDistanceFromCamera() {
        return getCameraToTag().getNorm();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> seesTag(), null);
        builder.addIntegerProperty("tag id", () -> lastTagId, null);
        builder.addDoubleProperty("tag distance m", () -> lastTagDistance, null);
    }
}