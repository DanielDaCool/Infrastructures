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
 * ({@link #HISTORY_LENGTH_SECONDS}) of odometry-only poses.</li>
 * <li>A vision frame taken at time t: the estimate is moved back to t by the odometry between
 * t and now, corrected there, and moved forward again by the same odometry. This is one step
 * per frame (the old version replayed the whole 1.5 s history for every frame, about 3 times).
 * Frames that arrive out of order are handled the same way, within a few mm of a full replay.</li>
 * <li><b>Trust follows the uncertainty</b> ({@code adaptiveTrust}, on by default): the estimator
 * keeps one variance P (m^2) for x and y. It grows with the distance driven and the time
 * ({@link #VARIANCE_PER_METER}, {@link #VARIANCE_PER_SECOND}) and jumps when the
 * slip/collision detector fires (see {@link #addOdometryData(List, Translation2d)}). A frame
 * with variance R pulls the estimate by {@code K = P / (P + R)}, and P becomes
 * {@code (1 - K) P}. So many frames don't pull harder than one good one, a long time without
 * vision makes the next frame count more, and a slip is fixed fast.</li>
 * <li><b>Jump filter:</b> a frame whose distance from the estimate is unlikely for P + R
 * (chi^2, 99.9 %) is rejected. If {@link #ESCAPE_REJECTED_FRAMES} of the last
 * {@link #ESCAPE_WINDOW_FRAMES} frames of one kind were rejected but agree with each other
 * within {@link #ESCAPE_SPREAD_METERS}, the estimate is the one that is wrong: P is raised to
 * cover them and the frame is used (for the Quest, its offset is re-aligned instead; see
 * {@link VisionResult#REALIGN_QUEST}). The filter is off while the robot is disabled (it may be
 * carried).</li>
 * <li>With {@code adaptiveTrust} off, P stays fixed at the state std devs and the gain is
 * WPILib's {@code q / (q + sqrt(q r))}; the jump filter and escape still work. Use this if the
 * slip/collision detector doesn't fire on the robot (README, push test).</li>
 * </ul>
 *
 * <p>The heading comes from the gyro. Vision only changes it if a measurement has a finite
 * heading std dev and the state heading std dev isn't 0 (none of the current sources measure
 * the heading).
 *
 * <p>Simulated on 10 fresh seeds x 17 scenarios: 6.2 cm mean position error with
 * {@code adaptiveTrust}, 9.6 cm with it off, 21.2 cm for the previous version (README).
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

    /**
     * Horizontal acceleration (m/s^2) above which the robot was hit. Driving can't get past about
     * 1.2 g (traction), hits are 5-40 g (the roboRIO accelerometer reads up to 8 g). Turning hard
     * with the roboRIO off center reads up to 2.5 g, and vibration spikes add to that, so 5 g
     * (3 g gave 5x more false hits in simulation). Tune from logs ("pose/acceleration g").
     */
    public static final double COLLISION_ACCELERATION = 5 * 9.81;
    /**
     * How long after the last hit sample the wheel motion into the hit is dropped (seconds),
     * only with {@code adaptiveTrust} off. A pin lasts 0.2-0.6 s in simulation (60 ms recovered
     * 16 % of the error, 0.15 s 44 %). Measure it: log a few hits and see how long the wheels
     * keep turning after the robot stopped.
     */
    public static final double COLLISION_HOLD_SECONDS = 0.15;

    /** Added to P (m^2) per meter driven. Tested range 0.0008-0.003, all within 0.3 cm. */
    public static final double VARIANCE_PER_METER = 0.0015;
    /** Added to P (m^2) per second. */
    public static final double VARIANCE_PER_SECOND = 1e-4;
    /** P never goes below this (1 cm)^2, so correlated frames can't make the estimate rigid. */
    public static final double MIN_VARIANCE = 1e-4;
    /** P after a reset, (10 cm)^2: a reset pose is close, but not exact. */
    public static final double RESET_VARIANCE = 0.01;
    /** Jump filter threshold: chi^2 with 2 degrees of freedom at 99.9 %. */
    public static final double GATE_CHI_SQUARED = 13.82;
    /** Rejected frames (of {@link #ESCAPE_WINDOW_FRAMES}) that make the estimate the one that is wrong. */
    public static final int ESCAPE_REJECTED_FRAMES = 15;
    /** How many recent frames of one kind the escape looks at. */
    public static final int ESCAPE_WINDOW_FRAMES = 25;
    /** The rejected frames must be within this distance (m) of their mean to count as agreeing. */
    public static final double ESCAPE_SPREAD_METERS = 0.3;

    /**
     * Slip: the difference between the wheel acceleration and the roboRIO's (m/s^2, smoothed)
     * above which the wheels are slipping or the robot is pushed.
     */
    public static final double SLIP_ACCELERATION = 1.5;
    /** Weight of the newest loop in the smoothed slip acceleration. */
    private static final double SLIP_SMOOTHING = 0.4;
    /** The slip error is forgotten after this long without slip (seconds). */
    public static final double SLIP_QUIET_SECONDS = 0.3;
    /** Added to P on a hit (m^2), (15 cm)^2. */
    public static final double COLLISION_VARIANCE = 0.15 * 0.15;
    /** A hit closer than this to the previous one is the same hit (seconds). */
    private static final double COLLISION_REARM_SECONDS = 0.1;
    /**
     * For this long after a hit (seconds), wheel motion into the hit direction faster than
     * {@link #PIN_SPEED} is distrusted (added to P): a pinned robot keeps spinning its wheels.
     */
    public static final double PIN_SECONDS = 2.0;
    /** See {@link #PIN_SECONDS} (m/s). */
    public static final double PIN_SPEED = 0.3;

    /** Odometry std devs and how vision measurements are combined; see {@link #setStateStd}. */
    private final double[] stateVarianceByAxis = new double[3];

    /** Which escape window a frame counts in: the cameras are pooled, the Quest is separate. */
    public enum MeasurementKind {
        CAMERA,
        QUEST
    }

    /** What happened to a vision measurement. */
    public enum VisionResult {
        /** Used. */
        ACCEPTED,
        /** Rejected by the jump filter. */
        REJECTED,
        /** Not looked at: NaN, outside the field, too old, too new or from before a reset. */
        DROPPED,
        /**
         * A Quest frame that started the escape: the Quest offset is the wrong one, so shift it
         * by {@link #getQuestRealignment()} instead of moving the estimate.
         */
        REALIGN_QUEST
    }

    private final DemaciaOdometry odometry;

    /** Odometry-only poses (the applied twists chained up) by FPGA time, plus P growth and yaw rate. */
    private final NavigableMap<Double, OdometryEntry> history = new TreeMap<>();
    /** The current fused pose. */
    private Pose2d estimatedPose = Pose2d.kZero;
    /** The odometry-only pose of the newest history entry. */
    private Pose2d odometryPose = Pose2d.kZero;
    /** Total variance added so far (growth and detector), to subtract what came after a late frame. */
    private double cumulativeVariance = 0;
    /** Position variance P (m^2), the same for x and y. */
    private double variance = RESET_VARIANCE;
    private boolean adaptiveTrust = true;
    private boolean enabled = true;
    /** Field size for the outside-the-field check (meters); infinite until set. */
    private double fieldLength = Double.POSITIVE_INFINITY;
    private double fieldWidth = Double.POSITIVE_INFINITY;
    /** Margin around the field for the outside-the-field check (meters). */
    public static final double FIELD_MARGIN_METERS = 0.5;

    /**
     * The last {@link #ESCAPE_WINDOW_FRAMES} frames of each kind: the distance to the estimate
     * of a rejected frame, or null for one that was used.
     */
    private final Map<MeasurementKind, List<Translation2d>> recentFrames = new EnumMap<>(MeasurementKind.class);
    private Translation2d questRealignment = Translation2d.kZero;
    private int rejectedFrames;
    private int escapes;

    // Slip/collision detector state; see detectSlipAndCollisions.
    /** FPGA time and odometry position of the newest sample of the previous loop, or null. */
    private double previousLoopTime = Double.NaN;
    private Translation2d previousLoopPosition;
    private Translation2d previousVelocity = Translation2d.kZero;
    private Translation2d smoothedSlipAcceleration = Translation2d.kZero;
    private Translation2d slipVelocity = Translation2d.kZero;
    /** Position error (m) the detector thinks the wheels have built up in this event. */
    private double slipError = 0;
    private double slipQuietSeconds = 0;
    /** FPGA time of the last sample over {@link #COLLISION_ACCELERATION}. */
    private double lastHitTime = Double.NEGATIVE_INFINITY;
    /** Unit vector (field frame) into the obstacle, from the last hit; null before the first. */
    private Translation2d pinDirection;
    private boolean slipping;
    private boolean pinned;

    /** FPGA time of the last hit for dropping the wheel motion into it ({@code adaptiveTrust} off). */
    private double lastDropHitTime = Double.NEGATIVE_INFINITY;
    /** Unit vector (gyro frame) the collision pushed the robot toward, taken from its first sample. */
    private Translation2d hitDirection = Translation2d.kZero;
    private boolean colliding = false;
    /**
     * FPGA time of the last {@link #resetPose}. Samples and frames measured before it are not
     * used: their motion happened before the reset pose, and after a gyro reset their gyro
     * reading can still be the old one (a sample already on its way from the odometry thread).
     */
    private double resetTimestampSeconds = Double.NEGATIVE_INFINITY;
    /**
     * Vision frames taken before this are dropped: the first odometry sample after the last
     * reset (the reset is only confirmed once a sample measured after it arrives).
     */
    private double visionResetTimestampSeconds = Double.NEGATIVE_INFINITY;
    /** True from a reset until the first sample after it. */
    private boolean waitingForSampleAfterReset = false;

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
     * Sets the odometry std devs. x and y are only used with {@code adaptiveTrust} off (then
     * P = std^2, fixed); theta is always used for vision that measures the heading. A 0 on an
     * axis means vision never changes that axis.
     *
     * @param stateSTD Std devs (x meters, y meters, theta radians).
     */
    public final void setStateStd(Matrix<N3, N1> stateSTD) {
        for (int i = 0; i < 3; ++i) {
            stateVarianceByAxis[i] = stateSTD.get(i, 0) * stateSTD.get(i, 0);
        }
    }

    /**
     * Turns trust that follows the uncertainty on (default) or off. Off, P stays fixed at the
     * state std devs and the wheel motion into a hit is dropped instead (like WPILib plus the
     * jump filter). Turn it off if the push test shows that the slip/collision flags don't fire.
     */
    public void setAdaptiveTrust(boolean adaptiveTrust) {
        this.adaptiveTrust = adaptiveTrust;
    }

    /** @return Whether trust follows the uncertainty; see {@link #setAdaptiveTrust}. */
    public boolean isAdaptiveTrust() {
        return adaptiveTrust;
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
     * Collision detection is off; see {@link #addOdometryData(List, Translation2d)}.
     */
    public void addOdometryData(List<OdometryData> samples) {
        addOdometryData(samples, Translation2d.kZero);
    }

    /**
     * Runs odometry on each sample (oldest first), moves the estimate by each twist and adds it
     * to the history, then runs the slip/collision detector once. Call once per loop with every
     * sample since the last loop, before adding that loop's vision.
     *
     * <p>A sample that isn't newer than the last one is moved just after it, so the history
     * stays in the order the twists were computed. P grows by
     * {@code VARIANCE_PER_METER * distance + VARIANCE_PER_SECOND * dt} per sample.
     *
     * <p><b>Slip/collision detector</b> (needs the acceleration; it changes P only with
     * {@code adaptiveTrust}, but its flags are always updated):
     * <ul>
     * <li>Slip: the wheel acceleration (from the odometry over the last two loops) minus the
     * roboRIO's, smoothed. Over {@link #SLIP_ACCELERATION} the wheels are slipping or the robot
     * is being pushed: the difference is integrated into a slip velocity, and the position error
     * it builds up is added to P. Forgotten after {@link #SLIP_QUIET_SECONDS} without slip.</li>
     * <li>Collision: over {@link #COLLISION_ACCELERATION} adds {@link #COLLISION_VARIANCE}.
     * For {@link #PIN_SECONDS} after it, wheel motion into the obstacle faster than
     * {@link #PIN_SPEED} is added to P too (the wheels of a pinned robot keep turning).</li>
     * </ul>
     *
     * <p>With {@code adaptiveTrust} off, a hit drops the part of the wheel motion that goes into
     * it for {@link #COLLISION_HOLD_SECONDS} instead; motion to the side or away is kept.
     *
     * @param samples                 Every odometry sample since the last loop, oldest first.
     * @param accelerationFromRoboRio Horizontal acceleration of the robot from the roboRIO's
     *                                built-in accelerometer, robot relative (x forward, y left,
     *                                m/s^2), with the part caused by spinning removed.
     *                                {@code Translation2d.kZero} turns the detector off.
     */
    public void addOdometryData(List<OdometryData> samples, Translation2d accelerationFromRoboRio) {
        if (samples.isEmpty()) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        boolean hasAcceleration = !accelerationFromRoboRio.equals(Translation2d.kZero);
        Rotation2d newestGyroAngle = samples.get(samples.size() - 1).gyroAngle();
        if (!adaptiveTrust && accelerationFromRoboRio.getNorm() > COLLISION_ACCELERATION) {
            if (now - lastDropHitTime >= COLLISION_HOLD_SECONDS) {
                // A new collision. Later samples of the same hit are the chassis ringing and can
                // point anywhere, so the direction is only taken here.
                hitDirection = accelerationFromRoboRio.rotateBy(newestGyroAngle)
                        .div(accelerationFromRoboRio.getNorm());
            }
            lastDropHitTime = now;
        }
        colliding = !adaptiveTrust && now - lastDropHitTime < COLLISION_HOLD_SECONDS;

        for (OdometryData sample : samples) {
            if (sample.timestampSeconds() < resetTimestampSeconds) {
                // Measured before the last reset: only move the module baseline.
                odometry.setModulePositions(sample.swerveModules());
                continue;
            }
            double timestamp = sample.timestampSeconds();
            if (waitingForSampleAfterReset) {
                visionResetTimestampSeconds = timestamp;
                waitingForSampleAfterReset = false;
            }
            double lastTimestamp = history.isEmpty() ? timestamp : history.lastKey();
            if (!history.isEmpty() && timestamp <= lastTimestamp) {
                timestamp = lastTimestamp + MIN_SAMPLE_SPACING_SECONDS;
            }
            Twist2d twist = odometry.updateOdometry(sample.gyroAngle(), sample.swerveModules());
            if (colliding) {
                twist = dropMotionIntoHit(twist, sample.gyroAngle());
            }
            double dt = timestamp - lastTimestamp;
            estimatedPose = estimatedPose.exp(twist);
            odometryPose = odometryPose.exp(twist);
            if (adaptiveTrust) {
                double added = VARIANCE_PER_METER * Math.hypot(twist.dx, twist.dy) + VARIANCE_PER_SECOND * dt;
                variance += added;
                cumulativeVariance += added;
            }
            history.put(timestamp, new OdometryEntry(odometryPose, cumulativeVariance,
                    dt > 0 ? twist.dtheta / dt : 0));
        }

        // Keep the newest entry older than the history length, so the history still covers it.
        if (!history.isEmpty()) {
            Double oldestNeeded = history.lowerKey(history.lastKey() - HISTORY_LENGTH_SECONDS);
            if (oldestNeeded != null) {
                history.headMap(oldestNeeded, false).clear();
            }
        }

        if (hasAcceleration) {
            detectSlipAndCollisions(now, accelerationFromRoboRio);
        } else {
            previousLoopPosition = null;
            slipping = false;
            pinned = false;
        }
    }

    /**
     * The slip/collision detector, once per loop (see {@link #addOdometryData(List, Translation2d)}).
     * Uses the raw wheel odometry (before anything is dropped).
     */
    private void detectSlipAndCollisions(double now, Translation2d accelerationRobotRelative) {
        if (history.isEmpty()) {
            return;
        }
        double loopTime = history.lastKey();
        Pose2d wheelPose = odometry.getOdometryPose();
        Translation2d position = wheelPose.getTranslation();
        if (previousLoopPosition == null) {
            previousLoopTime = loopTime;
            previousLoopPosition = position;
            return;
        }
        double dt = loopTime - previousLoopTime;
        Translation2d previousPosition = previousLoopPosition;
        previousLoopTime = loopTime;
        previousLoopPosition = position;
        if (dt <= 1e-6) {
            return;
        }
        Translation2d velocity = position.minus(previousPosition).div(dt);
        Translation2d wheelAcceleration = velocity.minus(previousVelocity).div(dt);
        previousVelocity = velocity;
        Translation2d acceleration = accelerationRobotRelative.rotateBy(wheelPose.getRotation());
        double accelerationNorm = acceleration.getNorm();

        double oldSlipError = slipError;
        double increase = 0;
        slipping = false;
        pinned = false;
        if (accelerationNorm > COLLISION_ACCELERATION) {
            if (now - lastHitTime > COLLISION_REARM_SECONDS) {
                addVariance(COLLISION_VARIANCE);
            }
            lastHitTime = now;
            pinDirection = acceleration.div(-accelerationNorm);
        }
        if (pinDirection != null && now - lastHitTime < PIN_SECONDS) {
            double speedIntoHit = velocity.getX() * pinDirection.getX() + velocity.getY() * pinDirection.getY();
            if (speedIntoHit > PIN_SPEED) {
                increase += speedIntoHit * dt;
                slipQuietSeconds = 0;
                pinned = true;
            }
        }
        if (accelerationNorm <= COLLISION_ACCELERATION) {
            smoothedSlipAcceleration = smoothedSlipAcceleration.times(1 - SLIP_SMOOTHING)
                    .plus(wheelAcceleration.minus(acceleration).times(SLIP_SMOOTHING));
            if (smoothedSlipAcceleration.getNorm() > SLIP_ACCELERATION) {
                slipVelocity = slipVelocity.plus(smoothedSlipAcceleration.times(dt));
                increase += slipVelocity.getNorm() * dt;
                slipQuietSeconds = 0;
                slipping = true;
            } else {
                slipVelocity = slipVelocity.times(0.5);
            }
        }
        if (increase > 0) {
            slipError = oldSlipError + increase;
            addVariance(slipError * slipError - oldSlipError * oldSlipError);
        } else {
            slipQuietSeconds += dt;
            if (slipQuietSeconds > SLIP_QUIET_SECONDS) {
                slipError = 0;
            }
        }
    }

    /**
     * Adds to P now; it counts as having happened at the newest history entry. Does nothing with
     * {@code adaptiveTrust} off (the detector still runs, for its flags).
     */
    private void addVariance(double addedVariance) {
        if (!adaptiveTrust) {
            return;
        }
        variance += addedVariance;
        cumulativeVariance += addedVariance;
        if (!history.isEmpty()) {
            history.lastEntry().getValue().cumulativeVariance = cumulativeVariance;
        }
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
     * {@link #FIELD_MARGIN_METERS} outside the field, it was taken before the first odometry
     * sample after the last reset, it is
     * older than the history, or more than {@link #MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS} newer
     * than the latest odometry sample (up to that, it is placed on the sample).</li>
     * <li>The estimate is moved back to the frame time by the odometry since then.</li>
     * <li>Jump filter and escape (see the class docs).</li>
     * <li>x and y are pulled by {@code K = P_t / (P_t + R)}, where P_t is P minus what was added
     * after the frame time (a late frame is compared with the uncertainty it had then). With
     * {@code adaptiveTrust} off, {@code K = q / (q + sqrt(q R))}.</li>
     * <li>The corrected pose is moved forward again by the same odometry.</li>
     * </ol>
     *
     * @param visionRobotPose  The measured field pose of the robot center.
     * @param timestampSeconds FPGA time the frame was captured (not the time it arrived).
     * @param stdDevs          Measurement std devs (x m, y m, theta rad). x and y should be
     *                         equal. {@code Double.POSITIVE_INFINITY} means "this axis was not
     *                         measured".
     * @param kind             Which escape window the frame counts in.
     * @return What was done with it.
     */
    public VisionResult addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs,
            MeasurementKind kind) {
        // A NaN once fused would stay in the pose for good.
        if (!isValidMeasurement(visionRobotPose, stdDevs) || !Double.isFinite(timestampSeconds)
                || timestampSeconds < visionResetTimestampSeconds || waitingForSampleAfterReset || history.isEmpty()
                || timestampSeconds < history.firstKey()
                || timestampSeconds > history.lastKey() + MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS
                || !isOnField(visionRobotPose.getTranslation())) {
            return VisionResult.DROPPED;
        }
        timestampSeconds = Math.min(timestampSeconds, history.lastKey());

        OdometryEntry atFrame = odometryAt(timestampSeconds);
        Pose2d estimateAtFrame = estimatedPose.plus(atFrame.pose.minus(odometryPose));
        double varianceAddedSince = cumulativeVariance - atFrame.cumulativeVariance;

        double residualX = visionRobotPose.getX() - estimateAtFrame.getX();
        double residualY = visionRobotPose.getY() - estimateAtFrame.getY();
        double measurementVarianceX = square(stdDevs.get(0, 0));
        double measurementVarianceY = square(stdDevs.get(1, 0));
        double varianceAtFrame = adaptiveTrust
                ? Math.max(variance - varianceAddedSince, MIN_VARIANCE) : stateVarianceByAxis[0];
        double fixedVarianceY = adaptiveTrust ? varianceAtFrame : stateVarianceByAxis[1];

        // Jump filter (an axis with infinite std dev adds 0); off while disabled.
        double chiSquared = residualX * residualX / (varianceAtFrame + measurementVarianceX)
                + residualY * residualY / (fixedVarianceY + measurementVarianceY);
        boolean rejected = enabled && chiSquared > GATE_CHI_SQUARED;
        List<Translation2d> window = recentFrames.get(kind);
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
                questRealignment = mean;
                return VisionResult.REALIGN_QUEST;
            }
            escapes++;
            if (adaptiveTrust) {
                varianceAtFrame = Math.max(varianceAtFrame, mean.getSquaredNorm());
                fixedVarianceY = varianceAtFrame;
            }
        }

        double gainX = gain(varianceAtFrame, measurementVarianceX);
        double gainY = gain(fixedVarianceY, measurementVarianceY);
        if (adaptiveTrust) {
            double averageGain = (gainX + gainY) / 2;
            variance = Math.max((1 - averageGain) * varianceAtFrame + varianceAddedSince, MIN_VARIANCE);
        }
        double gainTheta = fixedGain(stateVarianceByAxis[2], square(stdDevs.get(2, 0)));
        Pose2d corrected = new Pose2d(
                estimateAtFrame.getX() + gainX * residualX,
                estimateAtFrame.getY() + gainY * residualY,
                estimateAtFrame.getRotation().plus(new Rotation2d(gainTheta
                        * visionRobotPose.getRotation().minus(estimateAtFrame.getRotation()).getRadians())));
        estimatedPose = corrected.plus(odometryPose.minus(atFrame.pose));
        return VisionResult.ACCEPTED;
    }

    /** The x/y gain: Kalman with {@code adaptiveTrust}, WPILib's fixed formula without. */
    private double gain(double stateVariance, double measurementVariance) {
        if (!adaptiveTrust) {
            return fixedGain(stateVariance, measurementVariance);
        }
        if (Double.isInfinite(measurementVariance)) {
            return 0;
        }
        return stateVariance / (stateVariance + measurementVariance);
    }

    /** WPILib's gain {@code q / (q + sqrt(q r))}; 0 when q is 0 or r infinite. */
    private static double fixedGain(double stateVariance, double measurementVariance) {
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
     * The odometry-only pose and total added variance at a time inside the history, interpolated
     * between the two samples around it; clamped to the oldest/newest sample.
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
        OdometryEntry a = before.getValue();
        OdometryEntry b = after.getValue();
        return new OdometryEntry(a.pose.interpolate(b.pose, t),
                a.cumulativeVariance + t * (b.cumulativeVariance - a.cumulativeVariance), b.yawRate);
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
        Translation2d velocity = getOdometryMotion(newest - FUTURE_VELOCITY_WINDOW_SECONDS, newest)
                .div(newest - from);
        return new Pose2d(estimatedPose.getTranslation().plus(velocity.times(seconds)), estimatedPose.getRotation());
    }

    /** Time the velocity of {@link #getFuturePose} is averaged over (seconds). */
    private static final double FUTURE_VELOCITY_WINDOW_SECONDS = 0.1;

    /** @return The position std dev (meters): sqrt(P), or the state std dev with {@code adaptiveTrust} off. */
    public double getPositionStdDev() {
        return Math.sqrt(adaptiveTrust ? variance : stateVarianceByAxis[0]);
    }

    /** @return How far to shift the Quest after {@link VisionResult#REALIGN_QUEST} (field frame, meters). */
    public Translation2d getQuestRealignment() {
        return questRealignment;
    }

    /** @return True while the wheel motion into a hit is being dropped ({@code adaptiveTrust} off). */
    public boolean isColliding() {
        return colliding;
    }

    /** @return Whether the detector saw slip in the last loop. */
    public boolean isSlipping() {
        return slipping;
    }

    /** @return Whether the wheels pushed into the last hit in the last loop. */
    public boolean isPinned() {
        return pinned;
    }

    /** @return Seconds since the last hit over {@link #COLLISION_ACCELERATION}. */
    public double getSecondsSinceHit() {
        return Timer.getFPGATimestamp() - lastHitTime;
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
     * Resets the pose, clears the history and sets P to {@link #RESET_VARIANCE}.
     *
     * @param pose      The new field pose.
     * @param gyroAngle The raw gyro reading that corresponds to pose's heading.
     */
    public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
        odometry.resetPose(pose, gyroAngle);
        resetTimestampSeconds = Timer.getFPGATimestamp();
        waitingForSampleAfterReset = true;
        history.clear();
        estimatedPose = pose;
        odometryPose = pose;
        cumulativeVariance = 0;
        history.put(resetTimestampSeconds, new OdometryEntry(pose, 0, 0));
        variance = RESET_VARIANCE;
        for (List<Translation2d> window : recentFrames.values()) {
            window.clear();
        }
        previousLoopPosition = null;
        previousVelocity = Translation2d.kZero;
        slipError = 0;
        slipVelocity = Translation2d.kZero;
        lastHitTime = Double.NEGATIVE_INFINITY;
        lastDropHitTime = Double.NEGATIVE_INFINITY;
        pinDirection = null;
        colliding = false;
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

    /** One history entry: the odometry-only pose, the total variance added so far, the yaw rate. */
    private static final class OdometryEntry {
        private final Pose2d pose;
        /** Updated when the detector adds variance in the same loop. */
        private double cumulativeVariance;
        /** Yaw rate of the sample that ended here (rad/s). */
        private final double yawRate;

        private OdometryEntry(Pose2d pose, double cumulativeVariance, double yawRate) {
            this.pose = pose;
            this.cumulativeVariance = cumulativeVariance;
            this.yawRate = yawRate;
        }
    }
}
