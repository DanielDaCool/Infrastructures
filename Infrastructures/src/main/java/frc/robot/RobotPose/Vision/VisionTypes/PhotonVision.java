package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotPose.Vision.BaseVisionSource;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.visionConfigs.PhotonVisionConfig;

/**
 * A PhotonVision AprilTag camera. Each frame is solved with the coprocessor's multi-tag
 * result (enable MultiTag in the PhotonVision web UI), or with the lowest ambiguity single tag
 * when only one tag is seen. Only x and y are used; the gyro heading is better than a tag
 * heading.
 *
 * <p>Single-tag frames with an ambiguity above {@link #MAX_AMBIGUITY} are not used. The x/y std
 * dev grows with the tag distance and shrinks with the tag count (see
 * {@link BaseVisionSource#distanceStdDev}); the config std devs are not used.
 *
 * <p>The camera name in the config must match the name in the PhotonVision web UI.
 */
public class PhotonVision extends BaseVisionSource {
    /** Single-tag frames with a pose ambiguity above this are not used (PhotonVision suggests 0.2). */
    public static final double MAX_AMBIGUITY = 0.2;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    /** Measurements from the frames read in this loop's periodic(). */
    private final List<TimestampedVisionMeasurement> measurements = new ArrayList<>();
    /** Tags used in the newest used frame (for the dashboard). */
    private int tagCount;
    /** Single-tag frames rejected for ambiguity since startup (for the dashboard). */
    private int rejectedFrames;

    /** @param config The camera's config; {@code config.name} is the PhotonVision camera name. */
    public PhotonVision(PhotonVisionConfig config) {
        super(config);
        camera = new PhotonCamera(config.name);
        // PhotonVision takes robot to camera in the WPILib robot frame, the same frame as the offset.
        poseEstimator = new PhotonPoseEstimator(getAprilTagFieldLayout(), offset);
    }

    /**
     * Reads every frame that arrived since the last loop. {@code getAllUnreadResults} only
     * returns each frame once, so no frame is used twice.
     */
    @Override
    public void periodic() {
        super.periodic();
        measurements.clear();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result);
                if (estimate.isPresent()
                        && estimate.get().targetsUsed.get(0).getPoseAmbiguity() > MAX_AMBIGUITY) {
                    rejectedFrames++;
                    continue;
                }
            }
            estimate.ifPresent(this::addMeasurement);
        }
    }

    private void addMeasurement(EstimatedRobotPose estimate) {
        List<PhotonTrackedTarget> targets = estimate.targetsUsed;
        double distanceSum = 0;
        for (PhotonTrackedTarget target : targets) {
            distanceSum += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        tagCount = targets.size();
        double xyStd = distanceStdDev(distanceSum / tagCount, tagCount);
        pose = estimate.estimatedPose.toPose2d();
        measurements.add(new TimestampedVisionMeasurement(pose, estimate.timestampSeconds,
                VecBuilder.fill(xyStd, xyStd, Double.POSITIVE_INFINITY)));
    }

    /** @return Whether this loop's periodic() read at least one usable frame. */
    @Override
    public boolean shouldUpdate() {
        return !measurements.isEmpty();
    }

    /** @return The measurements read in this loop's periodic(), oldest first. */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        return List.copyOf(measurements);
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> shouldUpdate(), null);
        builder.addIntegerProperty("tag count", () -> tagCount, null);
        builder.addIntegerProperty("frames rejected (ambiguity)", () -> rejectedFrames, null);
    }
}
