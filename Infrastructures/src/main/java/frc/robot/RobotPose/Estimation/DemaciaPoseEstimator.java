package frc.robot.RobotPose.Estimation;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
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
 * Fuses odometry and vision into one field pose.
 *
 * <p><b>How it works:</b>
 * <ul>
 * <li>Each odometry sample moves the estimate by its twist and is stored in a short history
 * ({@link #HISTORY_LENGTH_SECONDS}) of odometry-only poses. While the robot is tilted (on a
 * bump or ramp), the wheel translation is trusted less (see {@link #MAX_TILT_RADIANS}).</li>
 * <li>A vision frame taken at time t: the estimate is moved back to t by the odometry between
 * t and now, corrected there, and moved forward again by the same odometry. This is one step
 * per frame (the old version replayed the whole 1.5 s history for every frame, about 3 times).
 * Frames that arrive out of order are handled the same way, within a few mm of a full replay.</li>
 * <li>The correction is WPILib's: per field axis, {@code K = q / (q + sqrt(q r))}, q = the
 * state variance (state std dev squared), r = the frame's variance.</li>
 * <li><b>Jump filter:</b> a frame whose distance from the estimate is unlikely for q + r
 * (chi^2, 99.9 %) is rejected. If {@link #ESCAPE_REJECTED_FRAMES} of the last
 * {@link #ESCAPE_WINDOW_FRAMES} frames of one kind were rejected but agree with each other
 * within {@link #ESCAPE_SPREAD_METERS}, the estimate is the one that is wrong and the frame is
 * used (for the Quest, its offset is re-aligned instead; see
 * {@link VisionResult#REALIGN_QUEST}). When camera frames escape, the Quest was anchored to the
 * wrong estimate too, so it is shifted by the same amount ({@link VisionResult#ESCAPED}). The
 * filter is off while the robot is disabled (it may be carried).</li>
 * </ul>
 *
 * <p>The heading comes from the gyro. Vision only changes it if a measurement has a finite
 * heading std dev and the state heading std dev isn't 0 (none of the current sources measure
 * the heading).
 *
 * <p>Simulated on 10 fresh seeds x 17 scenarios x 13 conditions: 8.6 cm mean position error,
 * against about 20 cm for the previous version (README).
 */
public class DemaciaPoseEstimator {

    /** How long the odometry history is kept. Vision frames older than this are dropped. */
    public static final double HISTORY_LENGTH_SECONDS = 1.5;

    /**
     * A vision frame up to this much newer than the latest odometry sample is placed on that
     * sample instead of being dropped. With high-frequency odometry the latest sample is a few
     * milliseconds old, so a fresh frame can be newer than it.
     */
    private static final double MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS = 0.05;

    /** Odometry samples closer together than this are spread apart to keep them in order. */
    private static final double MIN_SAMPLE_SPACING_SECONDS = 1e-6;

    /** A yaw rate from samples closer together than this (or out of order) keeps the previous rate. */
    private static final double MIN_YAW_RATE_DT_SECONDS = 1e-3;

    /**
     * At this tilt (radians) and more the wheels' translation is not used; below it, it is
     * scaled by {@code 1 - tilt / MAX_TILT_RADIANS} (6328's RobotState, 2026). On a bump the
     * wheels spin in the air or count the climb as floor distance. The heading is not affected.
     */
    public static final double MAX_TILT_RADIANS = Math.toRadians(25);

    /** Jump filter threshold: chi^2 with 2 degrees of freedom at 99.9 %. */
    public static final double GATE_CHI_SQUARED = 13.82;
    /** Rejected frames (of {@link #ESCAPE_WINDOW_FRAMES}) that make the estimate the one that is wrong. */
    public static final int ESCAPE_REJECTED_FRAMES = 15;
    /** How many recent frames of one kind the escape looks at. */
    public static final int ESCAPE_WINDOW_FRAMES = 25;
    /** The rejected frames must be within this distance (m) of their mean to count as agreeing. */
    public static final double ESCAPE_SPREAD_METERS = 0.3;
    /** Margin around the field for the outside-the-field check (meters). */
    public static final double FIELD_MARGIN_METERS = 0.5;

    /** Which escape window a frame counts in: the cameras are pooled, the Quest is separate. */
    public enum MeasurementKind {
        CAMERA,
        QUEST
    }

    /** What happened to a vision measurement. */
    public enum VisionResult {
        /** Used. */
        ACCEPTED,
        /**
         * A camera frame used after the escape: the estimate was wrong by about
         * {@link #getQuestRealignment()}, so shift the Quest (anchored to the old estimate) by it.
         */
        ESCAPED,
        /** Rejected by the jump filter. */
        REJECTED,
        /** Not looked at: NaN, outside the field, too old or too new. */
        DROPPED,
        /**
         * A Quest frame that started the escape: the Quest offset is the wrong one, so shift it
         * by {@link #getQuestRealignment()} instead of moving the estimate.
         */
        REALIGN_QUEST
    }

    private final DemaciaOdometry odometry;
    /** Per-axis (x meters, y meters, theta radians) squared state std devs, the "q" in the gain. */
    private final double[] stateVarianceByAxis = new double[3];

    /** Odometry-only poses (the applied twists chained up) and yaw rates by FPGA time. */
    private final NavigableMap<Double, OdometryEntry> history = new TreeMap<>();
    /** The current fused pose. */
    private Pose2d estimatedPose = Pose2d.kZero;
    /** The odometry-only pose of the newest history entry. */
    private Pose2d odometryPose = Pose2d.kZero;
    private boolean enabled = true;
    /** Field size for the outside-the-field check (meters); infinite until set. */
    private double fieldLength = Double.POSITIVE_INFINITY;
    private double fieldWidth = Double.POSITIVE_INFINITY;

    /**
     * The last {@link #ESCAPE_WINDOW_FRAMES} frames of each kind: the distance to the estimate
     * of a rejected frame, or null for one that was used.
     */
    private final Map<MeasurementKind, List<Translation2d>> recentFrames = new EnumMap<>(MeasurementKind.class);
    private Translation2d questRealignment = Translation2d.kZero;
    private int rejectedFrames;
    private int escapes;

    /**
     * FPGA time of the last {@link #resetPose}. Samples measured before it are not applied:
     * their motion happened before the reset pose, and after a gyro reset their gyro reading
     * can still be the old one (a sample already on its way from the odometry thread). Vision
     * frames from before it are older than the (cleared) history, so they are dropped too.
     */
    private double resetTimestampSeconds = Double.NEGATIVE_INFINITY;

    /** Raw time of the last sample (before it was moved to keep the order); NaN before the first. */
    private double lastSampleSeconds = Double.NaN;
    /** Yaw rate of the last sample; kept when the next one can't give a rate. */
    private double lastYawRate;
    /** True until the first sample after a reset: its twist spans the reset. */
    private boolean nextSampleSpansReset;
    /**
     * Raw time of the sample the first twist after the last reset started from (NaN if none),
     * so {@link #getFuturePose} divides that twist by its own time span.
     */
    private double resetTwistStartSeconds = Double.NaN;

    /**
     * @param moduleLocations  Module positions relative to the robot center (meters).
     * @param stateSTD         Odometry std devs (x m, y m, theta rad); see {@link #setStateStd}.
     */
    public DemaciaPoseEstimator(Translation2d[] moduleLocations, Matrix<N3, N1> stateSTD) {
        this.odometry = new DemaciaOdometry(moduleLocations);
        setStateStd(stateSTD);
        for (MeasurementKind kind : MeasurementKind.values()) {
            recentFrames.put(kind, new ArrayList<>());
        }
    }

    /**
     * Sets how much odometry is trusted, per axis. Together with each measurement's std devs
     * this decides how far a vision measurement pulls the pose, and how far off a frame can be
     * before the jump filter rejects it. A 0 on an axis means vision never changes that axis.
     *
     * @param stateSTD Std devs (x meters, y meters, theta radians).
     */
    public final void setStateStd(Matrix<N3, N1> stateSTD) {
        for (int i = 0; i < 3; ++i) {
            stateVarianceByAxis[i] = stateSTD.get(i, 0) * stateSTD.get(i, 0);
        }
    }

    /**
     * Tells the estimator whether the robot is enabled. While disabled the jump filter is off,
     * because the robot may be carried and every frame is then far from the estimate.
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Sets the field size; frames more than {@link #FIELD_MARGIN_METERS} outside it are
     * dropped. No check until this is called.
     */
    public void setFieldSize(double lengthMeters, double widthMeters) {
        fieldLength = lengthMeters;
        fieldWidth = widthMeters;
    }

    /**
     * Runs odometry on each sample (oldest first), moves the estimate by each twist and adds it
     * to the history. Call once per loop with every sample since the last loop, before adding
     * that loop's vision.
     *
     * <p>A sample that isn't newer than the last one is moved just after it, so the history
     * stays in the order the twists were computed. Its yaw rate is the rotation over the raw
     * time since the previous sample; if that is under {@link #MIN_YAW_RATE_DT_SECONDS} (or
     * negative), and for the first sample after a reset, the previous rate is kept. A sample's
     * translation is scaled by
     * {@code 1 - clamp(tilt / MAX_TILT_RADIANS, 0, 1)}; its rotation is kept.
     *
     * @param samples Every odometry sample since the last loop, oldest first.
     */
    public void addOdometryData(List<OdometryData> samples) {
        for (OdometryData sample : samples) {
            if (sample.timestampSeconds() < resetTimestampSeconds) {
                // Measured before the last reset: only move the module baseline.
                odometry.setModulePositions(sample.swerveModules());
                lastSampleSeconds = sample.timestampSeconds();
                continue;
            }
            double timestamp = sample.timestampSeconds();
            double lastTimestamp = history.isEmpty() ? timestamp : history.lastKey();
            if (!history.isEmpty() && timestamp <= lastTimestamp) {
                timestamp = lastTimestamp + MIN_SAMPLE_SPACING_SECONDS;
            }
            Twist2d twist = odometry.updateOdometry(sample.gyroAngle(), sample.swerveModules());
            double trust = tiltTrust(sample.tiltRadians());
            twist = new Twist2d(twist.dx * trust, twist.dy * trust, twist.dtheta);
            estimatedPose = estimatedPose.exp(twist);
            odometryPose = odometryPose.exp(twist);
            // Rate over the raw time since the previous sample (the moved timestamp can be 1 us).
            double dt = sample.timestampSeconds() - lastSampleSeconds;
            double yawRate = lastYawRate;
            if (nextSampleSpansReset) {
                // The twist started at the previous sample, before the reset: keep the rate.
                resetTwistStartSeconds = lastSampleSeconds;
                nextSampleSpansReset = false;
            } else if (dt >= MIN_YAW_RATE_DT_SECONDS) {
                yawRate = twist.dtheta / dt;
            }
            lastSampleSeconds = sample.timestampSeconds();
            lastYawRate = yawRate;
            history.put(timestamp, new OdometryEntry(odometryPose, yawRate));
        }

        // Keep the newest entry older than the history length, so the history still covers it.
        if (!history.isEmpty()) {
            Double oldestNeeded = history.lowerKey(history.lastKey() - HISTORY_LENGTH_SECONDS);
            if (oldestNeeded != null) {
                history.headMap(oldestNeeded, false).clear();
            }
        }
    }

    /** @return How much of the wheel translation to use at this tilt (1 level, 0 at 25 deg and more). */
    static double tiltTrust(double tiltRadians) {
        if (!(tiltRadians > 0)) {
            return 1;
        }
        return 1 - MathUtil.clamp(tiltRadians / MAX_TILT_RADIANS, 0, 1);
    }

    /**
     * Same as {@link #addVisionMeasurement(Pose2d, double, Matrix, MeasurementKind)} for a camera.
     */
    public VisionResult addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        return addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs, MeasurementKind.CAMERA);
    }

    /**
     * Adds a vision measurement taken at {@code timestampSeconds}.
     *
     * <ol>
     * <li>Dropped if the pose isn't finite, a std dev is NaN or negative, it is more than
     * {@link #FIELD_MARGIN_METERS} outside the field, it is older than the history (which a
     * reset clears), or more than {@link #MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS} newer than the
     * latest odometry sample (up to that, it is placed on the sample).</li>
     * <li>The estimate is moved back to the frame time by the odometry since then.</li>
     * <li>Jump filter and escape (see the class docs), unless the robot is disabled.</li>
     * <li>Each axis is pulled by {@code K = q / (q + sqrt(q r))}.</li>
     * <li>The corrected pose is moved forward again by the same odometry.</li>
     * </ol>
     *
     * @param visionRobotPose  The measured field pose of the robot center.
     * @param timestampSeconds FPGA time the frame was captured (not the time it arrived).
     * @param stdDevs          Measurement std devs (x m, y m, theta rad). 0 means "exactly
     *                         right"; {@code Double.POSITIVE_INFINITY} means "this axis was not
     *                         measured".
     * @param kind             Which escape window the frame counts in.
     * @return What was done with it.
     */
    public VisionResult addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs,
            MeasurementKind kind) {
        // A NaN once fused would stay in the pose for good.
        if (!isValidMeasurement(visionRobotPose, stdDevs) || !Double.isFinite(timestampSeconds)
                || history.isEmpty() || timestampSeconds < history.firstKey()
                || timestampSeconds > history.lastKey() + MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS
                || !isOnField(visionRobotPose.getTranslation())) {
            return VisionResult.DROPPED;
        }
        timestampSeconds = Math.min(timestampSeconds, history.lastKey());

        Pose2d odometryAtFrame = odometryAt(timestampSeconds).pose;
        Pose2d estimateAtFrame = estimatedPose.plus(odometryAtFrame.minus(odometryPose));
        double residualX = visionRobotPose.getX() - estimateAtFrame.getX();
        double residualY = visionRobotPose.getY() - estimateAtFrame.getY();
        double measurementVarianceX = square(stdDevs.get(0, 0));
        double measurementVarianceY = square(stdDevs.get(1, 0));

        VisionResult result = VisionResult.ACCEPTED;
        List<Translation2d> window = recentFrames.get(kind);
        if (!enabled) {
            window.clear();
        } else {
            // An axis with an infinite std dev adds 0.
            double chiSquared = residualX * residualX / (stateVarianceByAxis[0] + measurementVarianceX)
                    + residualY * residualY / (stateVarianceByAxis[1] + measurementVarianceY);
            boolean rejected = chiSquared > GATE_CHI_SQUARED;
            window.add(rejected ? new Translation2d(residualX, residualY) : null);
            if (window.size() > ESCAPE_WINDOW_FRAMES) {
                window.remove(0);
            }
            if (rejected) {
                rejectedFrames++;
                Translation2d mean = escapeMean(window);
                if (mean == null) {
                    return VisionResult.REJECTED;
                }
                window.clear();
                if (kind == MeasurementKind.QUEST) {
                    // The Quest frames are `mean` away from the estimate, so they move back by it.
                    questRealignment = mean.unaryMinus();
                    return VisionResult.REALIGN_QUEST;
                }
                escapes++;
                questRealignment = mean;
                result = VisionResult.ESCAPED;
            }
        }

        double gainX = gain(stateVarianceByAxis[0], measurementVarianceX);
        double gainY = gain(stateVarianceByAxis[1], measurementVarianceY);
        double gainTheta = gain(stateVarianceByAxis[2], square(stdDevs.get(2, 0)));
        Pose2d corrected = new Pose2d(
                estimateAtFrame.getX() + gainX * residualX,
                estimateAtFrame.getY() + gainY * residualY,
                estimateAtFrame.getRotation().plus(new Rotation2d(gainTheta
                        * visionRobotPose.getRotation().minus(estimateAtFrame.getRotation()).getRadians())));
        estimatedPose = corrected.plus(odometryPose.minus(odometryAtFrame));
        return result;
    }

    /** WPILib's gain {@code q / (q + sqrt(q r))}; 0 when q is 0 or r infinite. */
    private static double gain(double stateVariance, double measurementVariance) {
        if (stateVariance == 0.0 || Double.isInfinite(measurementVariance)) {
            return 0;
        }
        return stateVariance / (stateVariance + Math.sqrt(stateVariance * measurementVariance));
    }

    /**
     * @return The mean residual of the rejected frames in the window if there are at least
     *         {@link #ESCAPE_REJECTED_FRAMES} and all are within {@link #ESCAPE_SPREAD_METERS}
     *         of it, otherwise null.
     */
    private static Translation2d escapeMean(List<Translation2d> window) {
        double sumX = 0;
        double sumY = 0;
        int count = 0;
        for (Translation2d residual : window) {
            if (residual != null) {
                sumX += residual.getX();
                sumY += residual.getY();
                count++;
            }
        }
        if (count < ESCAPE_REJECTED_FRAMES) {
            return null;
        }
        Translation2d mean = new Translation2d(sumX / count, sumY / count);
        for (Translation2d residual : window) {
            if (residual != null && residual.getDistance(mean) > ESCAPE_SPREAD_METERS) {
                return null;
            }
        }
        return mean;
    }

    private static double square(double value) {
        return value * value;
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

    /** @return Whether the position is within the field plus {@link #FIELD_MARGIN_METERS}. */
    private boolean isOnField(Translation2d position) {
        return position.getX() >= -FIELD_MARGIN_METERS && position.getX() <= fieldLength + FIELD_MARGIN_METERS
                && position.getY() >= -FIELD_MARGIN_METERS && position.getY() <= fieldWidth + FIELD_MARGIN_METERS;
    }

    /**
     * The odometry-only pose at a time inside the history, interpolated between the two samples
     * around it; clamped to the oldest/newest sample.
     */
    private OdometryEntry odometryAt(double timestampSeconds) {
        var before = history.floorEntry(timestampSeconds);
        var after = history.higherEntry(timestampSeconds);
        if (before == null) {
            return history.firstEntry().getValue();
        }
        if (after == null) {
            return before.getValue();
        }
        double t = (timestampSeconds - before.getKey()) / (after.getKey() - before.getKey());
        return new OdometryEntry(before.getValue().pose.interpolate(after.getValue().pose, t), after.getValue().yawRate);
    }

    /** @return The current fused pose. */
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    /**
     * Returns the estimated pose at a past timestamp: the current estimate moved back by the
     * odometry since then. Clamped to the history (newer returns the latest pose).
     */
    public Pose2d getPoseAt(double timestampSeconds) {
        if (history.isEmpty()) {
            return estimatedPose;
        }
        return estimatedPose.plus(odometryAt(timestampSeconds).pose.minus(odometryPose));
    }

    /**
     * The motion between two times from the odometry alone, in the field frame of the estimate
     * (for the Quest's jump check). Clamped to the history.
     */
    public Translation2d getOdometryMotion(double fromTimestampSeconds, double toTimestampSeconds) {
        if (history.isEmpty()) {
            return Translation2d.kZero;
        }
        Rotation2d odometryToField = estimatedPose.getRotation().minus(odometryPose.getRotation());
        return odometryAt(toTimestampSeconds).pose.getTranslation()
                .minus(odometryAt(fromTimestampSeconds).pose.getTranslation()).rotateBy(odometryToField);
    }

    /**
     * @return The fastest yaw rate (rad/s, absolute) of the odometry samples from
     *         {@code windowSeconds} before the timestamp up to it (the first sample after the
     *         window start if there is none in it); 0 without history. For rejecting MegaTag2
     *         frames taken while spinning fast.
     */
    public double getPeakYawRate(double timestampSeconds, double windowSeconds) {
        if (history.isEmpty()) {
            return 0;
        }
        var window = history.subMap(timestampSeconds - windowSeconds, true, timestampSeconds, true);
        if (window.isEmpty()) {
            var next = history.ceilingEntry(timestampSeconds - windowSeconds);
            return next == null ? 0 : Math.abs(next.getValue().yawRate);
        }
        double peak = 0;
        for (OdometryEntry entry : window.values()) {
            peak = Math.max(peak, Math.abs(entry.yawRate));
        }
        return peak;
    }

    /**
     * The pose {@code seconds} from now if the robot keeps its current velocity: the estimate
     * plus the odometry velocity over the last 0.1 s (turned to the field frame) times seconds,
     * same heading. Odometry, not the estimate, so a vision correction doesn't read as speed.
     *
     * @param seconds How far ahead, clamped to [0, 1].
     */
    public Pose2d getFuturePose(double seconds) {
        seconds = MathUtil.clamp(seconds, 0, 1);
        if (history.size() < 2) {
            return estimatedPose;
        }
        double newest = history.lastKey();
        double from = Math.max(newest - FUTURE_VELOCITY_WINDOW_SECONDS, history.firstKey());
        // Right after a reset the first twist started at the sample before the reset.
        if (from == resetTimestampSeconds && resetTwistStartSeconds < from) {
            from = resetTwistStartSeconds;
        }
        Translation2d velocity = getOdometryMotion(newest - FUTURE_VELOCITY_WINDOW_SECONDS, newest)
                .div(newest - from);
        return new Pose2d(estimatedPose.getTranslation().plus(velocity.times(seconds)), estimatedPose.getRotation());
    }

    /** Time the velocity of {@link #getFuturePose} is averaged over (seconds). */
    private static final double FUTURE_VELOCITY_WINDOW_SECONDS = 0.1;

    /**
     * @return How far to shift the Quest frames (field frame, meters): after
     *         {@link VisionResult#REALIGN_QUEST} minus their mean distance from the estimate, after
     *         {@link VisionResult#ESCAPED} the cameras' mean distance from the old estimate.
     */
    public Translation2d getQuestRealignment() {
        return questRealignment;
    }

    /** @return How many frames the jump filter rejected since startup. */
    public int getRejectedFrames() {
        return rejectedFrames;
    }

    /** @return How many times the escape accepted rejected camera frames since startup. */
    public int getEscapes() {
        return escapes;
    }

    /**
     * Resets the pose and clears the history.
     *
     * @param pose      The new field pose.
     * @param gyroAngle The raw gyro reading that corresponds to pose's heading.
     */
    public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
        odometry.resetPose(pose, gyroAngle);
        resetTimestampSeconds = Timer.getFPGATimestamp();
        history.clear();
        estimatedPose = pose;
        odometryPose = pose;
        // The robot is still turning: keep the yaw rate; the next twist spans the reset.
        history.put(resetTimestampSeconds, new OdometryEntry(pose, lastYawRate));
        nextSampleSpansReset = true;
        resetTwistStartSeconds = Double.NaN;
        for (List<Translation2d> window : recentFrames.values()) {
            window.clear();
        }
    }


    /**
     * One odometry sample.
     *
     * @param timestampSeconds FPGA time the sample was measured.
     * @param gyroAngle        Raw gyro heading.
     * @param swerveModules    Module positions (total distance driven + wheel angle), same
     *                         order as the module locations.
     * @param tiltRadians      How far the robot is tilted from level (radians, 0 = level or
     *                         unknown); see {@link #tilt} and {@link #MAX_TILT_RADIANS}.
     */
    public record OdometryData(double timestampSeconds, Rotation2d gyroAngle, SwerveModulePosition[] swerveModules,
            double tiltRadians) {
        /** A level sample (no tilt). */
        public OdometryData(double timestampSeconds, Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
            this(timestampSeconds, gyroAngle, swerveModules, 0);
        }

        /** A level sample measured now (for odometry read in the robot loop, without a thread). */
        public OdometryData(Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
            this(Timer.getFPGATimestamp(), gyroAngle, swerveModules);
        }

        /**
         * @param pitchRadians Gyro pitch (e.g. {@code Math.toRadians(pigeon.getPitch().getValueAsDouble())}).
         * @param rollRadians  Gyro roll.
         * @return The angle between the robot's up axis and vertical: {@code acos(cos(pitch) cos(roll))}.
         */
        public static double tilt(double pitchRadians, double rollRadians) {
            return Math.acos(MathUtil.clamp(Math.cos(pitchRadians) * Math.cos(rollRadians), -1, 1));
        }
    }

    /** One history entry: the odometry-only pose and the yaw rate of the sample that ended here. */
    private static final class OdometryEntry {
        private final Pose2d pose;
        /** Yaw rate of the sample that ended here (rad/s). */
        private final double yawRate;

        private OdometryEntry(Pose2d pose, double yawRate) {
            this.pose = pose;
            this.yawRate = yawRate;
        }
    }
}
