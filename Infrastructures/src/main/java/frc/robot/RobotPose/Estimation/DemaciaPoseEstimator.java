package frc.robot.RobotPose.Estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/**
 * Fuses odometry and vision into one field pose. Uses the same approach as 6328 Mechanical
 * Advantage's PoseEstimator: a history of odometry twists that is replayed from a base pose
 * every time something changes.
 *
 * <p><b>How it works:</b>
 * <ul>
 * <li>{@code updates} maps a timestamp to that moment's odometry twist plus any vision
 * measurements taken at that moment.</li>
 * <li>Each odometry sample adds a new entry at the current time.</li>
 * <li>A vision measurement is placed at its capture time (usually in the past). If that
 * falls between two odometry entries, the later entry's twist is split in two at that time
 * so the measurement sits exactly where it was taken.</li>
 * <li>{@link #update()} starts from {@code initialPose} and applies every entry in time
 * order: move by the twist, then pull toward the vision measurements there. The result is
 * {@code latestPose}.</li>
 * <li>Entries older than {@link #HISTORY_LENGTH_SECONDS} are applied into
 * {@code initialPose} one last time and removed, so the history stays short.</li>
 * </ul>
 *
 * <p>Because everything after a late vision measurement is replayed on top of it, several
 * corrections add up correctly. (WPILib's estimator corrects each measurement against the
 * raw odometry instead.)
 */
public class DemaciaPoseEstimator {

    /**
     * How long an entry stays in the history before it is folded into
     * {@code initialPose}. Vision measurements older than this are dropped.
     */
    public static final double HISTORY_LENGTH_SECONDS = 1.5;

    /**
     * A vision frame up to this much newer than the latest odometry sample is placed on that
     * sample instead of being dropped. With high-frequency odometry the latest sample is a few
     * milliseconds old, so a fresh frame can be newer than it.
     */
    private static final double MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS = 0.05;

    /** Odometry samples closer together than this are spread apart to keep them in order. */
    private static final double MIN_SAMPLE_SPACING_SECONDS = 1e-6;

    /**
     * Horizontal acceleration (m/s^2) above which the robot was hit. Driving can't get past about
     * 1.2 g (traction), hits are 5-40 g (the roboRIO accelerometer reads up to 8 g). 3 g leaves
     * room for tilt (0.26 g on the 15 deg bump) and turning the robot hard.
     * TODO tune from logs ("pose/acceleration g").
     */
    public static final double COLLISION_ACCELERATION = 3 * 9.81;
    /**
     * How long after the last hit sample the collision lasts (seconds): the wheels take ~25 ms
     * to spin down after the robot stops, plus about one loop, since the accelerometer is read
     * once per loop.
     */
    public static final double COLLISION_HOLD_SECONDS = 0.06;

    private final DemaciaOdometry odometry;

    /** Time of the last sample over {@link #COLLISION_ACCELERATION}. */
    private double lastHitTime = Double.NEGATIVE_INFINITY;
    /** Unit vector (gyro frame) the collision pushed the robot toward, taken from its first sample. */
    private Translation2d hitDirection = Translation2d.kZero;
    private boolean colliding = false;
    /**
     * FPGA time of the last {@link #resetPose}. Samples measured before it are not applied:
     * their motion happened before the reset pose, and after a gyro reset their gyro reading
     * can still be the old one (a sample already on its way from the odometry thread).
     */
    private double resetTimestampSeconds = Double.NEGATIVE_INFINITY;

    /** Per-axis (x meters, y meters, theta radians) squared state std devs, used as the "q" term in the per-axis gain formula below. */
    private final double[] stateVarianceByAxis = new double[3];

    /** The pose just before the oldest entry in {@code updates}; replay starts from here. */
    private Pose2d initialPose = new Pose2d();
    /** Result of the last replay: the current fused pose. */
    private Pose2d latestPose = new Pose2d();
    /** FPGA timestamp to odometry twist and vision measurements at that time, sorted by time. */
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();

    /**
     * @param moduleLocations  Module positions relative to the robot center (meters).
     * @param stateSTD         Odometry std devs (x m, y m, theta rad); see {@link #setStateStd}.
     */
    public DemaciaPoseEstimator(Translation2d[] moduleLocations, Matrix<N3, N1> stateSTD) {
        this.odometry = new DemaciaOdometry(moduleLocations);
        setStateStd(stateSTD);
        this.latestPose = this.initialPose;
    }

    /**
     * Sets how much odometry is trusted, per axis. Together with each measurement's std devs
     * this decides how far a vision measurement pulls the pose (see {@code PoseUpdate.apply}).
     * A 0 on an axis means vision never changes that axis.
     *
     * @param stateSTD Std devs (x meters, y meters, theta radians).
     */
    public final void setStateStd(Matrix<N3, N1> stateSTD) {
        for (int i = 0; i < 3; ++i) {
            stateVarianceByAxis[i] = stateSTD.get(i, 0) * stateSTD.get(i, 0);
        }
    }

    /**
     * Runs odometry on each sample (oldest first), stores each twist at its sample's
     * timestamp, then replays the history once. Call once per loop with every sample since
     * the last loop, before adding that loop's vision.
     *
     * <p>A sample that isn't newer than the last one is moved just after it, so the history
     * stays in the order the twists were computed.
     *
     * <p>Collision detection is off; see {@link #addOdometryData(List, Translation2d)}.
     */
    public void addOdometryData(List<OdometryData> samples) {
        addOdometryData(samples, Translation2d.kZero);
    }

    /**
     * Same as {@link #addOdometryData(List)}, with collision detection.
     *
     * <p><b>Collisions:</b> a loop with horizontal acceleration over
     * {@link #COLLISION_ACCELERATION} is a hit, and the collision lasts until
     * {@link #COLLISION_HOLD_SECONDS} after the last one. While colliding, the part of the wheel
     * motion that goes into the hit is dropped: driving into a wall or robot, the wheels keep
     * turning while the robot is stopped. Motion to the side of or away from the hit is kept
     * (being hit from the side or behind, the wheels report too little, and dropping it would
     * make that worse). The heading change from the gyro is always kept.
     *
     * <p>The acceleration is read once per loop, so the hit is checked once (at the current
     * FPGA time, with the newest sample's gyro angle) and applies to every sample of the loop.
     *
     * <p>Not fixed here (only vision can): being shoved while stopped, pushing that goes on
     * after the hit (no acceleration left to see), and hits shorter than the samples we read.
     *
     * @param samples                 Every odometry sample since the last loop, oldest first.
     * @param accelerationFromRoboRio Horizontal acceleration of the robot from the roboRIO's
     *                                built-in accelerometer, robot relative (x forward, y left,
     *                                m/s^2), with the part caused by spinning removed.
     *                                {@code Translation2d.kZero} turns collision detection off.
     */
    public void addOdometryData(List<OdometryData> samples, Translation2d accelerationFromRoboRio) {
        if (samples.isEmpty()) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        Rotation2d newestGyroAngle = samples.get(samples.size() - 1).gyroAngle();
        if (accelerationFromRoboRio.getNorm() > COLLISION_ACCELERATION) {
            if (now - lastHitTime >= COLLISION_HOLD_SECONDS) {
                // A new collision. Later samples of the same hit are the chassis ringing and can
                // point anywhere, so the direction is only taken here.
                hitDirection = accelerationFromRoboRio.rotateBy(newestGyroAngle)
                        .div(accelerationFromRoboRio.getNorm());
            }
            lastHitTime = now;
        }
        colliding = now - lastHitTime < COLLISION_HOLD_SECONDS;

        for (OdometryData sample : samples) {
            if (sample.timestampSeconds() < resetTimestampSeconds) {
                // Measured before the last reset: only move the module baseline.
                odometry.setModulePositions(sample.swerveModules());
                continue;
            }
            double timestamp = sample.timestampSeconds();
            if (!updates.isEmpty() && timestamp <= updates.lastKey()) {
                timestamp = updates.lastKey() + MIN_SAMPLE_SPACING_SECONDS;
            }
            Twist2d twist = odometry.updateOdometry(sample.gyroAngle(), sample.swerveModules());
            if (colliding) {
                twist = dropMotionIntoHit(twist, sample.gyroAngle());
            }
            updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
        }
        update();
    }

    /**
     * Removes the part of the twist's translation that goes into the hit (against
     * {@link #hitDirection}, the direction the robot was pushed). The part to the side or away
     * from the hit, and the rotation, are kept.
     *
     * @param twist     Robot-relative motion of one sample.
     * @param gyroAngle Gyro angle of that sample, to turn the hit direction to the robot frame.
     */
    private Twist2d dropMotionIntoHit(Twist2d twist, Rotation2d gyroAngle) {
        Translation2d hitRobot = hitDirection.rotateBy(gyroAngle.unaryMinus());
        // Negative when the motion goes into the hit.
        double alongHit = twist.dx * hitRobot.getX() + twist.dy * hitRobot.getY();
        if (alongHit >= 0) {
            return twist;
        }
        return new Twist2d(twist.dx - alongHit * hitRobot.getX(), twist.dy - alongHit * hitRobot.getY(),
                twist.dtheta);
    }

    /**
     * Adds a vision measurement at the time it was captured and replays the history.
     *
     * <ul>
     * <li>Dropped if the pose isn't finite or a std dev is NaN or negative.</li>
     * <li>If an entry already exists at exactly that time, the measurement is added to it
     * (and fused with the others there).</li>
     * <li>Otherwise the odometry entry right after the timestamp is split into two twists
     * (before/after the capture time, in proportion to time), and the measurement is put
     * on the new entry in between.</li>
     * <li>If it is newer than the latest odometry sample by up to
     * {@link #MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS}, it is added to that sample.</li>
     * <li>Dropped if it is older than the whole history or further ahead of the latest
     * odometry sample.</li>
     * </ul>
     *
     * @param visionRobotPose  The measured field pose of the robot center.
     * @param timestampSeconds FPGA time the frame was captured (not the time it arrived).
     * @param stdDevs          Measurement std devs (x m, y m, theta rad). 0 means "exactly
     *                         right"; {@code Double.POSITIVE_INFINITY} means "this axis was
     *                         not measured".
     */
    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        // A NaN once fused is replayed every loop and then folded into the base pose, so it would never go away.
        if (!isValidMeasurement(visionRobotPose, stdDevs)) {
            return;
        }

        VisionUpdate visionUpdate = new VisionUpdate(visionRobotPose, stdDevs);

        if (!updates.isEmpty() && timestampSeconds > updates.lastKey()
                && timestampSeconds - updates.lastKey() <= MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS) {
            timestampSeconds = updates.lastKey();
        }

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

    /** Pose must be finite; std devs must be non-negative and not NaN (infinite means "not measured"). */
    private static boolean isValidMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs) {
        if (!Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getRotation().getRadians())) {
            return false;
        }
        for (int i = 0; i < 3; i++) {
            double std = stdDevs.get(i, 0);
            if (Double.isNaN(std) || std < 0) {
                return false;
            }
        }
        return true;
    }

    /**
     * Folds entries older than {@link #HISTORY_LENGTH_SECONDS} into {@code initialPose}
     * (always keeping at least one), then replays everything left to get {@code latestPose}.
     */
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

    /** @return The current fused pose (from the last replay). */
    public Pose2d getEstimatedPose() {
        return latestPose;
    }

    /** @return True while a collision is limiting the wheel translation. */
    public boolean isColliding() {
        return colliding;
    }

    /**
     * Returns the estimated pose at a past timestamp: the history is replayed up to it and the
     * odometry twist that spans it is interpolated. Timestamps newer than the latest odometry
     * sample return the latest pose; older than the history return the oldest pose.
     */
    public Pose2d getPoseAt(double timestampSeconds) {
        Pose2d pose = initialPose;
        double prevKey = Double.NaN;
        for (var entry : updates.entrySet()) {
            double key = entry.getKey();
            if (key <= timestampSeconds) {
                pose = entry.getValue().apply(pose, stateVarianceByAxis);
                prevKey = key;
                continue;
            }
            if (!Double.isNaN(prevKey)) {
                Twist2d twist = entry.getValue().twist;
                double frac = (timestampSeconds - prevKey) / (key - prevKey);
                pose = pose.exp(new Twist2d(twist.dx * frac, twist.dy * frac, twist.dtheta * frac));
            }
            break;
        }
        return pose;
    }

    /**
     * Resets the pose and clears the whole history (including pending vision).
     *
     * @param pose      The new field pose.
     * @param gyroAngle The raw gyro reading that corresponds to pose's heading.
     */
    public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
        odometry.resetPose(pose, gyroAngle);
        updates.clear();
        initialPose = pose;
        latestPose = pose;
        lastHitTime = Double.NEGATIVE_INFINITY;
        colliding = false;
        resetTimestampSeconds = Timer.getFPGATimestamp();
    }


    /**
     * One odometry sample.
     *
     * @param timestampSeconds FPGA time the sample was measured.
     * @param gyroAngle        Raw gyro heading.
     * @param swerveModules    Module positions (total distance driven + wheel angle), same
     *                         order as the module locations.
     */
    public record OdometryData(double timestampSeconds, Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
        /** A sample measured now (for odometry read in the robot loop, without a thread). */
        public OdometryData(Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
            this(Timer.getFPGATimestamp(), gyroAngle, swerveModules);
        }
    }

    /** One vision measurement waiting in the history. */
    private static final class VisionUpdate {
        private final Pose2d pose;
        private final Matrix<N3, N1> stdDevs;
        private VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.stdDevs = stdDevs;
        }
    }

    /** One history entry: the odometry motion up to this time, then the vision taken at it. */
    private static final class PoseUpdate {
        private final Twist2d twist;
        private final List<VisionUpdate> visionUpdates;
        private PoseUpdate(Twist2d twist, List<VisionUpdate> visionUpdates) {
            this.twist = twist;
            this.visionUpdates = visionUpdates;
        }

        /**
         * Moves {@code lastPose} by the twist, then corrects it toward this entry's vision.
         *
         * <p>Per field axis (x, y, theta), separately:
         * <ol>
         * <li>Fuse all measurements here into one: inverse-variance weighted mean of the
         * residuals (measured minus estimate). If any has std 0, the plain mean of those is
         * used instead. If all are infinite, the axis is left alone.</li>
         * <li>Gain {@code K = q / (q + sqrt(q * r))}, where q is the odometry variance and r
         * the fused measurement variance (same formula WPILib uses). K = 1 means jump to the
         * measurement, K = 0 means ignore it.</li>
         * <li>{@code estimate += K * residual}.</li>
         * </ol>
         */
        private Pose2d apply(Pose2d lastPose, double[] stateVarianceByAxis) {
            Pose2d pose = lastPose.exp(twist);
            if (visionUpdates.isEmpty()) {
                return pose;
            }

            // Measurements with the same timestamp are fused (inverse-variance) into one before
            // the gain is applied, so N measurements of variance r count as one of variance r/N.
            double[] residualByAxis = new double[3];
            double[] measurementVarianceByAxis = new double[3];
            for (int axis = 0; axis < 3; axis++) {
                double informationSum = 0;
                double weightedResidualSum = 0;
                int exactCount = 0;
                double exactResidualSum = 0;
                for (VisionUpdate visionUpdate : visionUpdates) {
                    double std = visionUpdate.stdDevs.get(axis, 0);
                    double variance = std * std;
                    double residual = residual(axis, visionUpdate.pose, pose);
                    if (variance == 0.0) {
                        exactCount++;
                        exactResidualSum += residual;
                    } else {
                        informationSum += 1.0 / variance;
                        weightedResidualSum += residual / variance;
                    }
                }
                if (exactCount > 0) {
                    residualByAxis[axis] = exactResidualSum / exactCount;
                    measurementVarianceByAxis[axis] = 0.0;
                } else if (informationSum == 0.0) {
                    // Every std dev on this axis is infinite: nothing was measured on it.
                    residualByAxis[axis] = 0.0;
                    measurementVarianceByAxis[axis] = Double.POSITIVE_INFINITY;
                } else {
                    residualByAxis[axis] = weightedResidualSum / informationSum;
                    measurementVarianceByAxis[axis] = 1.0 / informationSum;
                }
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

            // The gain is applied per field axis (the frame the std devs are given in), not to
            // pose.log(vision), whose dx/dy are robot-relative and bent by dtheta.
            return new Pose2d(
                    pose.getX() + kalmanGainByAxis[0] * residualByAxis[0],
                    pose.getY() + kalmanGainByAxis[1] * residualByAxis[1],
                    pose.getRotation().plus(new Rotation2d(kalmanGainByAxis[2] * residualByAxis[2])));
        }

        /** Field-frame residual of one axis (x, y in meters; theta wrapped to [-pi, pi]). */
        private static double residual(int axis, Pose2d measured, Pose2d estimate) {
            switch (axis) {
                case 0:
                    return measured.getX() - estimate.getX();
                case 1:
                    return measured.getY() - estimate.getY();
                default:
                    return measured.getRotation().minus(estimate.getRotation()).getRadians();
            }
        }
    }
}