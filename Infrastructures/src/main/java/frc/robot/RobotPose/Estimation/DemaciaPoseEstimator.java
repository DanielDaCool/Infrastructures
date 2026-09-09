package frc.robot.RobotPose.Estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/**
 * Pose estimator using the twist-sequence + splice + full-replay mechanism
 * (matching the approach used by Mechanical-Advantage's (6328) PoseEstimator.java because WPILib's class has an acknowledged limitation
 * where multiple vision corrections do not compound - each corrects against raw odometry
 * rather than the previously-corrected estimate). 
 */
public class DemaciaPoseEstimator {

    /**
     * How long a twist/vision-correction entry is kept in the active update map before
     * being folded into basePose.
     */
    public static final double HISTORY_LENGTH_SECONDS = 1.5;

    private final DemaciaOdometry odometry;

    /** Per-axis (x meters, y meters, theta radians) squared state std devs, used as the "q" term in the per-axis gain formula below. */
    private final double[] stateVarianceByAxis = new double[3];

    private Pose2d initialPose = new Pose2d();
    private Pose2d latestPose = new Pose2d();
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();

    public DemaciaPoseEstimator(SwerveModulePosition[] initialPositions, Matrix<N3, N1> stateSTD) {
        this.odometry = new DemaciaOdometry(initialPositions);
        setStateStd(stateSTD);
        this.latestPose = this.initialPose;
    }

    public final void setStateStd(Matrix<N3, N1> stateSTD) {
        for (int i = 0; i < 3; ++i) {
            stateVarianceByAxis[i] = stateSTD.get(i, 0) * stateSTD.get(i, 0);
        }
    }

    public void addOdometryData(OdometryData odometryData) {
        double timestamp = Timer.getFPGATimestamp();
        Twist2d twist = odometry.updateOdometry(odometryData.gyroAngle(), odometryData.swerveModules());
        updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
        update();
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        VisionUpdate visionUpdate = new VisionUpdate(visionRobotPose, stdDevs);

        if (updates.containsKey(timestampSeconds)) {
            updates.get(timestampSeconds).visionUpdates.add(visionUpdate);
            update();
            return;
        }

        var prevEntry = updates.floorEntry(timestampSeconds);
        var nextEntry = updates.ceilingEntry(timestampSeconds);

        if (prevEntry == null || nextEntry == null) {
            // No bracketing "next" entry (or no "prev" entry, e.g. timestamp predates
            // everything currently tracked) - silently drop, per explicit decision.
            return;
        }

        double prevKey = prevEntry.getKey();
        double nextKey = nextEntry.getKey();
        PoseUpdate nextUpdate = nextEntry.getValue();

        double frac0 = (timestampSeconds - prevKey) / (nextKey - prevKey);
        double frac1 = (nextKey - timestampSeconds) / (nextKey - prevKey);

        Twist2d twist0 = new Twist2d(nextUpdate.twist.dx * frac0, nextUpdate.twist.dy * frac0,
                nextUpdate.twist.dtheta * frac0);
        Twist2d twist1 = new Twist2d(nextUpdate.twist.dx * frac1, nextUpdate.twist.dy * frac1,
                nextUpdate.twist.dtheta * frac1);

        List<VisionUpdate> newEntryVisionUpdates = new ArrayList<>();
        newEntryVisionUpdates.add(visionUpdate);

        updates.put(timestampSeconds, new PoseUpdate(twist0, newEntryVisionUpdates));
        updates.put(nextKey, new PoseUpdate(twist1, nextUpdate.visionUpdates));

        update();
    }

    private void update() {
        double now = Timer.getFPGATimestamp();

        while (updates.size() > 1 && updates.firstKey() < now - HISTORY_LENGTH_SECONDS) {
            var oldestEntry = updates.pollFirstEntry();
            initialPose = oldestEntry.getValue().apply(initialPose, stateVarianceByAxis);
        }

        Pose2d pose = initialPose;
        for (PoseUpdate poseUpdate : updates.values()) {
            pose = poseUpdate.apply(pose, stateVarianceByAxis);
        }
        latestPose = pose;
    }

    public Pose2d getEstimatedPose() {
        return latestPose;
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
        updates.clear();
        initialPose = pose;
        latestPose = pose;
    }


    public record OdometryData(Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
    }

    private static final class VisionUpdate {
        private final Pose2d pose;
        private final Matrix<N3, N1> stdDevs;
        private VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.stdDevs = stdDevs;
        }
    }

    private static final class PoseUpdate {
        private final Twist2d twist;
        private final List<VisionUpdate> visionUpdates;
        private PoseUpdate(Twist2d twist, List<VisionUpdate> visionUpdates) {
            this.twist = twist;
            this.visionUpdates = visionUpdates;
        }

        private Pose2d apply(Pose2d lastPose, double[] stateVarianceByAxis) {
            Pose2d pose = lastPose.exp(twist);
            for (VisionUpdate visionUpdate : visionUpdates) {
                double[] measurementVarianceByAxis = new double[3];
                Matrix<N3, N1> stdDevs = visionUpdate.stdDevs;
                for (int i = 0; i < 3; i++) {
                    measurementVarianceByAxis[i] = stdDevs.get(i, 0) * stdDevs.get(i, 0);
                }
                double[] kalmanGainByAxis = new double[3];
                for (int row = 0; row < 3; row++) {
                    if (stateVarianceByAxis[row] == 0.0) {
                        kalmanGainByAxis[row] = 0.0;
                    } else {
                        kalmanGainByAxis[row] = stateVarianceByAxis[row]
                                / (stateVarianceByAxis[row] + Math.sqrt(stateVarianceByAxis[row] * measurementVarianceByAxis[row]));
                    }
                }
                Twist2d visionTwist = pose.log(visionUpdate.pose);
                Twist2d scaledTwist = new Twist2d(visionTwist.dx * kalmanGainByAxis[0],
                        visionTwist.dy * kalmanGainByAxis[1], visionTwist.dtheta * kalmanGainByAxis[2]);
                pose = pose.exp(scaledTwist);
            }
            return pose;
        }
    }
}