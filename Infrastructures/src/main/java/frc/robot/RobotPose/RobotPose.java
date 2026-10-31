package frc.robot.RobotPose;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.MeasurementKind;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.VisionResult;
import frc.robot.RobotPose.Estimation.PhoenixOdometryThread;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionConfig;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera3d;
import frc.robot.RobotPose.Vision.VisionTypes.Quest;

/**
 * The robot's field pose. This is the only class the rest of the robot code should use
 * to read or reset the pose.
 *
 * <p>Every loop {@link #periodic()} feeds the {@link DemaciaPoseEstimator} with:
 * <ol>
 * <li>whether the robot is enabled (the estimator's jump filter is off while disabled),</li>
 * <li>every odometry sample (gyro + swerve module positions) since the last loop, from the
 * supplier passed to {@link #initialize} (one sample at 50 Hz, about five with a 250 Hz
 * {@link PhoenixOdometryThread}), and</li>
 * <li>every new measurement from the {@link VisionSource}s in the {@link VisionConfig}.</li>
 * </ol>
 *
 * <p>Vision sources are handled generically through the {@link VisionSource} interface. The
 * only exception is {@link Quest}: the Quest reports poses relative to the pose it was last
 * set to, so RobotPose re-anchors it (see {@link Quest#setPose}) whenever it reconnects and
 * whenever the pose is reset.
 *
 * <p>Singleton: call {@link #initialize} once (in RobotContainer, after
 * the chassis is created), then {@link #getInstance()} everywhere else.
 */
public final class RobotPose implements Sendable {

    private static RobotPose instance;

    private final DemaciaPoseEstimator poseEstimator;
    /** Returns every odometry sample since the last call; called once per loop. */
    private final Supplier<List<OdometryData>> odometrySamplesSupplier;
    /** Raw gyro reading of the newest odometry sample. */
    private Rotation2d lastGyroAngle = Rotation2d.kZero;
    /** Writes a new yaw to the gyro; used by {@link #setYaw}. */
    private final Consumer<Rotation2d> gyroYawSetter;
    /**
     * Writes the new heading to the gyro on {@link #resetPose}, or null to only store an offset
     * in the estimator; see {@link #initialize}.
     */
    private final Consumer<Rotation2d> poseResetGyroSetter;
    /** Every configured vision source, including the Quest if there is one. */
    private final List<VisionSource> sources;
    /** The estimated pose on the dashboard ({@code pose/field}). */
    private final Field2d field = new Field2d();

    /** A Quest and a 3D Limelight further apart than this (meters) disagree. */
    private static final double SOURCE_DISAGREEMENT_METERS = 0.3;
    /** How long they have to disagree before the alert is raised (seconds). */
    private static final double SOURCE_DISAGREEMENT_SECONDS = 1.0;
    /** One disagreement check per Quest and 3D Limelight pair, by "quest name/limelight name". */
    private final Map<String, SourceDisagreement> sourceDisagreements = new HashMap<>();

    private RobotPose(Supplier<List<OdometryData>> odometrySamplesSupplier, Consumer<Rotation2d> gyroYawSetter,
        Consumer<Rotation2d> poseResetGyroSetter, Translation2d[] moduleLocations, Matrix<N3, N1> stateStd,
        List<VisionSource> sources) {
        this.odometrySamplesSupplier = odometrySamplesSupplier;
        this.gyroYawSetter = gyroYawSetter;
        this.poseResetGyroSetter = poseResetGyroSetter;
        this.poseEstimator = new DemaciaPoseEstimator(moduleLocations, stateStd);
        this.sources = sources;
        AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        poseEstimator.setFieldSize(field.getFieldLength(), field.getFieldWidth());

        addLog();
    }

    /**
     * Dashboard entries: {@code pose} (x, y, heading), {@code pose/field}, and buttons to
     * reset the gyro to 0 or 180 degrees and the pose to the origin.
     */
    private void addLog() {
        PoseLogger.get().putData("pose", this);
        PoseLogger.get().putData("pose/field", field);
        PoseLogger.get().putData("pose/reset pose",
                new InstantCommand(() -> resetPose(Pose2d.kZero)).ignoringDisable(true));
        PoseLogger.get().putData("chassis/reset gyro",
                new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        PoseLogger.get().putData("chassis/reset gyro 180",
                new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
    }

        /**
     * Runs one loop of pose estimation. Called from {@code Robot.robotPeriodic()}, after the
     * CommandScheduler.
     *
     * <ol>
     * <li>Adds this loop's odometry samples. This always happens first, so vision frames
     * captured up to now have odometry around them to be placed into.</li>
     * <li>Calls {@link VisionSource#periodic()} on every source. Sources read their device
     * here (and the 3D Limelight sends it the current heading).</li>
     * <li>For each source: if it is a Quest that just reconnected, re-anchors it to the
     * current estimate and skips it this loop; otherwise, if {@link VisionSource#shouldUpdate()}
     * is true, adds each of its measurements to the estimator.</li>
     * </ol>
     *
     * Steps 2 and 3 are separate loops so that every source has read its device before any
     * measurement is used. Before the measurements are added, every Quest is compared with every
     * 3D Limelight (see {@link #checkQuestAgainstLimelights}).
     */
    public void periodic() {
        poseEstimator.setEnabled(DriverStation.isEnabled());
        addNewOdometrySamples();

        for (VisionSource source : sources) {
            source.periodic();
        }

        Map<VisionSource, List<TimestampedVisionMeasurement>> measurementsBySource = new HashMap<>();
        for (VisionSource source : sources) {
            if (source instanceof Quest && ((Quest) source).hasDrifted()) {
                ((Quest) source).setPose(poseEstimator.getEstimatedPose());
            }
            else if (source.shouldUpdate()) {
                measurementsBySource.put(source, source.getPoseEstimates());
            }
        }

        checkQuestAgainstLimelights(measurementsBySource);

        for (VisionSource source : sources) {
            for (TimestampedVisionMeasurement measurement : measurementsBySource.getOrDefault(source, List.of())) {
                double errorMeters = measurement.pose().getTranslation().getDistance(
                        poseEstimator.getPoseAt(measurement.timestampSeconds()).getTranslation());
                source.onMeasurementUsed(measurement, errorMeters);
                VisionResult result = poseEstimator.addVisionMeasurement(measurement.pose(),
                        measurement.timestampSeconds(), measurement.stdDevs(),
                        source instanceof Quest ? MeasurementKind.QUEST : MeasurementKind.CAMERA);
                if (result == VisionResult.REALIGN_QUEST) {
                    ((Quest) source).shiftOffset(poseEstimator.getQuestRealignment());
                } else if (result == VisionResult.ESCAPED) {
                    // The cameras showed the estimate was wrong; the Quests were anchored to it.
                    for (VisionSource quest : sources) {
                        if (quest instanceof Quest) {
                            ((Quest) quest).shiftOffset(poseEstimator.getQuestRealignment());
                        }
                    }
                }
            }
        }

        field.setRobotPose(poseEstimator.getEstimatedPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("x", () -> getEstimatedPose().getX(), null);
        builder.addDoubleProperty("y", () -> getEstimatedPose().getY(), null);
        builder.addDoubleProperty("heading deg", () -> getEstimatedPose().getRotation().getDegrees(), null);
        builder.addIntegerProperty("frames rejected", poseEstimator::getRejectedFrames, null);
        builder.addIntegerProperty("escapes", poseEstimator::getEscapes, null);
    }

    /**
     * When a Quest and a 3D Limelight both have a new measurement this loop, puts the distance
     * between their newest ones on {@code vision/<limelight name>/<quest name> distance m}, and
     * raises an alert if it stays over {@link #SOURCE_DISAGREEMENT_METERS} for
     * {@link #SOURCE_DISAGREEMENT_SECONDS}. The two frames are usually taken at different times,
     * so the Quest's position is first moved by the odometry between the two capture times. A
     * lasting disagreement means one of the offsets is wrong (or the Quest lost tracking without
     * saying so). The alert clears when the Quest disconnects.
     */
    private void checkQuestAgainstLimelights(Map<VisionSource, List<TimestampedVisionMeasurement>> measurementsBySource) {
        for (VisionSource quest : sources) {
            if (!(quest instanceof Quest)) {
                continue;
            }
            TimestampedVisionMeasurement questMeasurement = newest(measurementsBySource.get(quest));
            for (VisionSource limelight : sources) {
                if (!(limelight instanceof LimelightTagCamera3d)) {
                    continue;
                }
                SourceDisagreement disagreement = sourceDisagreements.computeIfAbsent(
                        quest.getName() + "/" + limelight.getName(),
                        key -> new SourceDisagreement(quest.getName(), limelight.getName()));
                TimestampedVisionMeasurement limelightMeasurement = newest(measurementsBySource.get(limelight));
                if (questMeasurement != null && limelightMeasurement != null) {
                    Translation2d motionBetweenFrames = poseEstimator.getPoseAt(limelightMeasurement.timestampSeconds())
                            .getTranslation().minus(poseEstimator.getPoseAt(questMeasurement.timestampSeconds()).getTranslation());
                    double distance = questMeasurement.pose().getTranslation().plus(motionBetweenFrames)
                            .getDistance(limelightMeasurement.pose().getTranslation());
                    disagreement.distanceMeters = distance;
                    disagreement.update(distance > SOURCE_DISAGREEMENT_METERS);
                } else if (!quest.isConnected()) {
                    disagreement.update(false);
                }
            }
        }
    }

    /** @return The measurement with the latest capture time, or null if there is none. */
    private static TimestampedVisionMeasurement newest(List<TimestampedVisionMeasurement> measurements) {
        TimestampedVisionMeasurement newest = null;
        if (measurements != null) {
            for (TimestampedVisionMeasurement measurement : measurements) {
                if (newest == null || measurement.timestampSeconds() > newest.timestampSeconds()) {
                    newest = measurement;
                }
            }
        }
        return newest;
    }

    /**
     * Alert and dashboard entry ({@code vision/<limelight>/<quest>/distance m}) for one Quest and
     * 3D Limelight pair; see {@link #checkQuestAgainstLimelights}.
     */
    private static final class SourceDisagreement {
        private final Debouncer debouncer = new Debouncer(SOURCE_DISAGREEMENT_SECONDS, DebounceType.kRising);
        private final String alertText;
        private double distanceMeters = Double.NaN;

        private SourceDisagreement(String questName, String limelightName) {
            alertText = questName + " and " + limelightName + " disagree by more than "
                    + SOURCE_DISAGREEMENT_METERS + " m";
            PoseLogger.get().putData("vision/" + limelightName + "/" + questName,
                    builder -> builder.addDoubleProperty("distance m", () -> distanceMeters, null));
        }

        private void update(boolean disagrees) {
            PoseLogger.get().alert(alertText, AlertType.kWarning, debouncer.calculate(disagrees));
        }
    }

    /** Adds every odometry sample since the last call to the estimator. */
    private void addNewOdometrySamples() {
        List<OdometryData> samples = odometrySamplesSupplier.get();
        if (!samples.isEmpty()) {
            poseEstimator.addOdometryData(samples);
            lastGyroAngle = samples.get(samples.size() - 1).gyroAngle();
        }
    }

    /**
     * @return The latest fused field pose (blue-alliance origin, meters and radians), as of
     *         the last {@link #periodic()}.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPose();
    }

    /**
     * The estimated pose at a past FPGA timestamp (e.g. a vision frame's capture time). Only
     * the last {@link DemaciaPoseEstimator#HISTORY_LENGTH_SECONDS} are kept; older timestamps
     * return the oldest kept pose.
     *
     * @param timestampSeconds FPGA time, in seconds.
     */
    public Pose2d getEstimatedPoseAt(double timestampSeconds) {
        return poseEstimator.getPoseAt(timestampSeconds);
    }

    /**
     * The pose {@code seconds} from now if the robot keeps its current velocity, taken from the
     * odometry (a vision correction doesn't count as speed). Same heading as now.
     *
     * @param seconds How far ahead, clamped to [0, 1].
     */
    public Pose2d getFuturePose(double seconds) {
        return poseEstimator.getFuturePose(seconds);
    }

    /**
     * The fastest yaw rate (rad/s) of the odometry samples in the {@code windowSeconds} before
     * a timestamp (to reject MegaTag2 frames taken while spinning).
     */
    public double getPeakYawRate(double timestampSeconds, double windowSeconds) {
        return poseEstimator.getPeakYawRate(timestampSeconds, windowSeconds);
    }

    /** The odometry-only motion between two FPGA times, in the field frame (meters). */
    public Translation2d getOdometryMotion(double fromTimestampSeconds, double toTimestampSeconds) {
        return poseEstimator.getOdometryMotion(fromTimestampSeconds, toTimestampSeconds);
    }

    /**
     * Sets the robot's pose (position and heading). The vision history is cleared and every
     * Quest is re-anchored to the new pose.
     *
     * <p>If a pose reset gyro setter was passed to {@link #initialize}, the new heading is also
     * written to the gyro (like {@link #setYaw}). Otherwise the gyro itself is not changed; the
     * estimator stores the difference between the gyro and the new heading instead, so anything
     * that reads the gyro directly (e.g. field-relative driving) must use
     * {@code getEstimatedPose().getRotation()} instead.
     *
     * @param pose The new field pose.
     */
    public void resetPose(Pose2d pose) {
        // Samples measured before the reset must not be applied on top of it.
        addNewOdometrySamples();
        if (poseResetGyroSetter != null) {
            // The setter can block (a Pigeon 2 waits for the ack): the motion measured from here
            // on happened after the new pose, so it is kept.
            double askedSeconds = Timer.getFPGATimestamp();
            poseResetGyroSetter.accept(pose.getRotation());
            // The gyro was just set, so pass the new heading as the gyro reading (the new value
            // may not have been read back from the device yet).
            resetEstimator(pose, pose.getRotation(), askedSeconds);
        } else {
            resetEstimator(pose, lastGyroAngle, Double.POSITIVE_INFINITY);
        }
    }

    /**
     * Sets the heading only, keeping the current position. Unlike {@link #resetPose}, this
     * also writes the new angle to the gyro. Bound to the "chassis/reset gyro" and
     * "chassis/reset gyro 180" dashboard buttons.
     *
     * @param angle The new field heading; {@code null} does nothing.
     */
    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            // Samples measured before the reset must not be applied on top of it.
            addNewOdometrySamples();
            gyroYawSetter.accept(angle);
            // The gyro was just set to `angle`, so pass it as the gyro reading (the new value may
            // not have been read back from the device yet). The position is kept, so the
            // translation of every sample not added yet (e.g. measured while the setter blocked)
            // is kept too.
            resetEstimator(new Pose2d(getEstimatedPose().getTranslation(), angle), angle, Double.NEGATIVE_INFINITY);
        }
    }

    /**
     * Resets the estimator, adds the samples that arrived since (see
     * {@link DemaciaPoseEstimator#resetPose(Pose2d, Rotation2d, double)}) and re-anchors every
     * Quest to the resulting pose. The Quest reports poses relative to its last anchor, so
     * without this its next frames pull the estimate back to the old pose.
     *
     * <p>The odometry thread keeps measuring meanwhile, but it only hands samples over through
     * {@link #addNewOdometrySamples} (a locked drain), so the estimator stays on the calling
     * thread. A sample of the block that is handed over later is kept the same way by
     * {@link #periodic()}: the estimator sorts samples by their timestamps, not by arrival.
     */
    private void resetEstimator(Pose2d pose, Rotation2d gyroAngle, double keepTranslationFromSeconds) {
        poseEstimator.resetPose(pose, gyroAngle, keepTranslationFromSeconds);
        addNewOdometrySamples(); // e.g. the samples measured while the gyro setter blocked
        Pose2d anchor = poseEstimator.getEstimatedPose();
        for (VisionSource source : sources) {
            if (source instanceof Quest) {
                ((Quest) source).setPose(anchor);
            }
        }
    }

    /**
     * @return The raw gyro reading of the newest odometry sample (as of the last
     *         {@link #periodic()}). This is not the field heading if the pose has been reset
     *         without setting the gyro; use {@link #getEstimatedPose()} for that.
     */
    public Rotation2d getGyroAngle() {
        return lastGyroAngle;
    }

    /**
     * Creates the singleton. Call once from RobotContainer, after the chassis is created.
     * A second call replaces the instance (and its history). {@link #resetPose} does not write
     * the gyro; use the overload with a pose reset gyro setter for that.
     *
     * @param odometrySamples        Returns every odometry sample (gyro angle and swerve
     *                               module positions) since the last call, oldest first;
     *                               called every loop. Use
     *                               {@code odometryThread::getNewSamples} for 250 Hz
     *                               odometry, or
     *                               {@code () -> List.of(new OdometryData(gyro, modules))}
     *                               to read once per loop.
     * @param gyroYawSetter          Writes a new yaw to the gyro (used by {@link #setYaw}),
     *                               e.g. {@code angle -> gyro.setYaw(angle.getDegrees())}.
     * @param moduleLocations        Each module's position relative to the robot center
     *                               (meters), in the same order as the module positions.
     * @param stateStd               How much to trust odometry, per axis (x m, y m, theta rad).
     *                               Larger values let vision correct the pose faster and let
     *                               frames further off through the jump filter. A 0 on an axis
     *                               means vision never changes that axis.
     * @param visionConfig           The vision sources to use. They are already created when
     *                               the config is built.
     */
    public static synchronized void initialize(Supplier<List<OdometryData>> odometrySamples,
            Consumer<Rotation2d> gyroYawSetter, Translation2d[] moduleLocations, Matrix<N3, N1> stateStd,
            VisionConfig visionConfig) {
        initialize(odometrySamples, gyroYawSetter, null, moduleLocations, stateStd, visionConfig);
    }

    /**
     * Same as {@link #initialize(Supplier, Consumer, Translation2d[], Matrix, VisionConfig)}, but
     * {@link #resetPose} also writes the new heading to the gyro.
     *
     * @param poseResetGyroSetter Writes the new heading to the gyro on {@link #resetPose}, e.g.
     *                            {@code angle -> gyro.setYaw(angle.getDegrees())}. {@code null}
     *                            keeps the gyro unchanged (only an offset is stored).
     */
    public static synchronized void initialize(Supplier<List<OdometryData>> odometrySamples,
            Consumer<Rotation2d> gyroYawSetter, Consumer<Rotation2d> poseResetGyroSetter,
            Translation2d[] moduleLocations, Matrix<N3, N1> stateStd, VisionConfig visionConfig) {
        instance = new RobotPose(odometrySamples,
            gyroYawSetter,
            poseResetGyroSetter,
            moduleLocations,
            stateStd,
            visionConfig.getSources());
    }

    /**
     * Returns the singleton instance, or null if initialize(...) has not been
     * called yet.
     */
    public static synchronized RobotPose getInstance() {
        return instance;
    }
}