package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;
import java.util.function.DoubleFunction;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotPose.PoseLogger;
import frc.robot.RobotPose.RobotPose;
import frc.robot.RobotPose.Vision.BaseVisionSource;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.visionConfigs.QuestConfig;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * A Meta Quest headset running QuestNav (https://questnav.gg). The Quest tracks its own
 * motion, and reports its pose relative to the last pose it was set to with
 * {@link #setPose}. So it has to be <b>anchored</b> to the fused estimate before its poses
 * mean anything on the field.
 *
 * <p>State:
 * <ul>
 * <li>{@code hasQuestDisconnected}: true at startup and after a disconnect, until the next
 * {@link #setPose}. While it is true and the Quest is connected, {@link #hasDrifted()} is
 * true and RobotPose re-anchors it.</li>
 * <li>{@code hasUpdatedQuestIntialPose}: set once a tracking frame arrives that was captured
 * at least {@link #QUEST_RESET_SETTLE_SECONDS} after the last {@link #setPose}. Only then
 * does {@link #shouldUpdate()} allow its measurements.</li>
 * </ul>
 *
 * <p>Every unread frame of the loop that is tracking and was captured after the settle time
 * goes through a {@link QuestFrameFilter} (jumps and lost tracking: ignore and re-align;
 * thinned to one frame per {@code QuestConfig.periodSeconds}), and each frame it keeps becomes
 * a measurement at its own capture time: QuestNav's timestamp (publish time) minus
 * {@code QuestConfig.latencySeconds}. Its position is converted from the headset to the robot
 * center with the config offset; the heading is copied from the estimate at the frame's time,
 * so only x/y are measured (theta is sent with an infinite std dev, like the Limelights). The
 * x/y std dev is {@code QuestConfig.thinnedStd} (or the config std, if bigger).
 *
 * <p>A disconnect only counts after {@link #DISCONNECT_DEBOUNCE_SECONDS}, so a short Wi-Fi
 * hiccup doesn't re-anchor the Quest.
 *
 * <p>The newest frames can be a few ms newer than the newest odometry sample (RobotPose adds
 * odometry before reading vision). The estimator adds a frame up to 50 ms newer than that
 * sample to the sample itself, and drops frames further ahead
 * (see {@code DemaciaPoseEstimator.addVisionMeasurement}).
 */
public class Quest extends BaseVisionSource {
    private final QuestNav questNav;
    private final QuestConfig config;
    private final QuestFrameFilter frameFilter;
    /** Connected, with drops shorter than {@link #DISCONNECT_DEBOUNCE_SECONDS} ignored. */
    private final Debouncer connectedDebouncer;

    /** The Quest must stay disconnected this long before it is re-anchored (seconds). */
    public static final double DISCONNECT_DEBOUNCE_SECONDS = 0.5;

    /** This loop's measurements, oldest first; see {@link #toMeasurements}. */
    private List<TimestampedVisionMeasurement> measurements = List.of();

    /** True once frames from after the last anchor are arriving; see class docs. */
    private boolean hasUpdatedQuestIntialPose;
    /** True at startup and after a disconnect, until re-anchored; see class docs. */
    private boolean hasQuestDisconnected;
    /** Whether the newest frame read was tracking. */
    private boolean isTracking;
    /** Whether this loop's periodic() got at least one usable frame. */
    private boolean hasNewPose;
    /** FPGA time of the last {@link #setPose}; infinite until the first one. */
    private double poseResetTimestamp = Double.POSITIVE_INFINITY;

    /**
     * QuestNav does not acknowledge setPose(), so frames are only trusted once they were
     * captured this long after the reset was sent.
     */
    private static final double QUEST_RESET_SETTLE_SECONDS = 0.25;

    /** Battery percent at or below which {@link #lowBatteryText} is raised. */
    private static final int LOW_BATTERY_PERCENT = 20;
    /**
     * Text of the low battery alert, raised while the Quest is connected, has sent its device
     * data, and its battery is at or below {@link #LOW_BATTERY_PERCENT}. Updated every loop, so it
     * clears after a battery swap.
     */
    private final String lowBatteryText = name + " battery is at or below " + LOW_BATTERY_PERCENT + " percent";
    /**
     * QuestNav's device data topic, only to see whether it has ever been received. QuestNav's
     * battery percent reads 0 (not empty) until then.
     */
    private final GenericSubscriber deviceDataSubscriber = NetworkTableInstance.getDefault()
            .getTable("QuestNav").getTopic("deviceData").genericSubscribe();

    /**
     * @param config The Quest's config. offset is robot center to headset:
     *               {@code robotPose = questPose.transformBy(offset.inverse())} and
     *               {@code questPose = robotPose.transformBy(offset)}. The name is only used
     *               for the dashboard.
     */
    public Quest(QuestConfig config) {
        this(config, new QuestNav());
    }

    /**
     * Same as {@link #Quest(QuestConfig)}, with a given QuestNav (e.g. a fake one in tests).
     *
     * @param config   The Quest's config.
     * @param questNav The QuestNav to read; it is used only by this source.
     */
    public Quest(QuestConfig config, QuestNav questNav) {
        super(config);
        this.config = config;
        frameFilter = new QuestFrameFilter(config.jumpMeters, config.gapSeconds, config.blackoutSeconds,
                config.periodSeconds);
        connectedDebouncer = new Debouncer(DISCONNECT_DEBOUNCE_SECONDS, DebounceType.kFalling);
        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = true;
        this.questNav = questNav;
        questNav.commandPeriodic();
    }

    /**
     * Adds a "Reset Quest Pose" dashboard button that re-anchors the Quest to the current
     * estimate. (It used to anchor the Quest to (0, 0, 0) without moving the estimate, so the
     * Quest then pulled the estimate toward the origin. To move the robot's pose, use
     * {@code RobotPose.resetPose}, which also re-anchors the Quest.)
     */
    @Override
    protected void addLog() {
        super.addLog();
        PoseLogger.get().putData("vision/" + getName() + "/Reset Quest Pose", new InstantCommand(
            ()->setPose(RobotPose.getInstance().getEstimatedPose())).ignoringDisable(true));
    }

    /**
     * @return True if the Quest is connected, has been anchored and settled, and this loop
     *         got at least one usable frame.
     */
    @Override
    public boolean shouldUpdate() {
        return isConnected() && hasUpdatedQuestIntialPose && hasNewPose;
    }

    /**
     * @return One measurement per usable frame of this loop, oldest first, with the config
     *         x/y std devs. The theta std dev is infinite because the heading was copied from
     *         the estimate, not measured. Only valid when {@link #shouldUpdate()} is true.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        return measurements;
    }

    @Override
    public boolean isConnected() {
        return questNav != null && questNav.isConnected();
    }

    public int getBattery() {
        if (questNav == null) {
            return 0;
        }
        return questNav.getBatteryPercent().orElse(0);
    }

    /**
     * Runs QuestNav's own update (it must run every loop), reads all unread frames, and turns
     * every usable one into a measurement (see {@link #toMeasurements}). {@link #pose} is the
     * newest of them. Also updates the anchor state: marks the Quest settled once a frame from
     * after the settle time arrives, and marks it disconnected (so it gets re-anchored) if the
     * connection drops. Then updates the battery alert.
     */
    @Override
    public void periodic() {
        super.periodic();

        questNav.commandPeriodic();

        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            isTracking = poseFrames[poseFrames.length - 1].isTracking();
        }

        RobotPose robotPose = RobotPose.getInstance();
        measurements = toMeasurements(poseFrames, offset, Math.max(std.get(0, 0), config.thinnedStd),
                config.latencySeconds, poseResetTimestamp + QUEST_RESET_SETTLE_SECONDS, frameFilter,
                robotPose::getEstimatedPoseAt, robotPose::getOdometryMotion);
        hasNewPose = !measurements.isEmpty();

        if (hasNewPose) {
            pose = measurements.get(measurements.size() - 1).pose();
            if (!hasQuestDisconnected) {
                hasUpdatedQuestIntialPose = true;
            }
        }

        if (!connectedDebouncer.calculate(isConnected()) && !hasQuestDisconnected) {
            hasQuestDisconnected = true;
            hasUpdatedQuestIntialPose = false;
        }

        PoseLogger.get().alert(lowBatteryText, AlertType.kWarning, isBatteryLow(isConnected(),
                deviceDataSubscriber.getLastChange() != 0, questNav.getBatteryPercent()));
    }

    /**
     * Turns QuestNav frames into measurements of the robot center. A frame is used only if it
     * is tracking, was captured after {@code settledAfterSeconds}, and the filter keeps it.
     *
     * @param frames              Frames as QuestNav returns them, oldest first.
     * @param offset              Robot center to headset (the config offset).
     * @param xyStd               x/y std dev of the measurements; theta is infinite.
     * @param latencySeconds      Subtracted from QuestNav's (publish time) timestamp.
     * @param settledAfterSeconds Last anchor time plus {@link #QUEST_RESET_SETTLE_SECONDS}
     *                            (infinite before the first anchor, so nothing is used).
     * @param filter              Jump handling and thinning; moves the position onto the fused pose.
     * @param estimateAt          The estimated pose at an FPGA time (heading and re-alignment).
     * @param odometryMotion      The odometry motion between two FPGA times (jump check).
     * @return One measurement per used frame, in the same order, each at its own capture time.
     */
    static List<TimestampedVisionMeasurement> toMeasurements(PoseFrame[] frames, Transform3d offset,
            double xyStd, double latencySeconds, double settledAfterSeconds, QuestFrameFilter filter,
            DoubleFunction<Pose2d> estimateAt, QuestFrameFilter.OdometryMotion odometryMotion) {
        List<TimestampedVisionMeasurement> result = new ArrayList<>();
        for (PoseFrame frame : frames) {
            double timestamp = frame.dataTimestamp() - latencySeconds;
            // Written so a NaN timestamp is skipped too.
            if (!frame.isTracking() || !(timestamp > settledAfterSeconds)) {
                continue;
            }
            Translation2d questPosition = frame.questPose3d().transformBy(offset.inverse())
                    .toPose2d().getTranslation();
            Translation2d position = filter.filter(timestamp, questPosition,
                    t -> estimateAt.apply(t).getTranslation(), odometryMotion);
            if (position == null) {
                continue;
            }
            Rotation2d heading = estimateAt.apply(timestamp).getRotation();
            result.add(new TimestampedVisionMeasurement(new Pose2d(position, heading), timestamp,
                    VecBuilder.fill(xyStd, xyStd, Double.POSITIVE_INFINITY)));
        }
        return result;
    }

    /**
     * Shifts where the Quest's frames are placed on the field, after the estimator found that
     * they agree with each other but not with the estimate (see
     * {@code DemaciaPoseEstimator.VisionResult.REALIGN_QUEST}).
     */
    public void shiftOffset(Translation2d shift) {
        frameFilter.shiftOffset(shift);
    }

    /**
     * @param connected         Whether the Quest is connected.
     * @param deviceDataArrived Whether QuestNav's device data has been received at least once.
     *                          Before that QuestNav reports a battery of 0.
     * @param battery           QuestNav's battery percent.
     * @return Whether the low battery alert should be on.
     */
    static boolean isBatteryLow(boolean connected, boolean deviceDataArrived, OptionalInt battery) {
        return connected && deviceDataArrived && battery.isPresent()
                && battery.getAsInt() <= LOW_BATTERY_PERCENT;
    }

    /**
     * @return True if the Quest is connected but hasn't been anchored since startup or since
     *         it last disconnected. RobotPose then calls {@link #setPose} with the estimate.
     */
    public boolean hasDrifted() {
        return hasQuestDisconnected && isConnected();
    }

    /**
     * Anchors the Quest: tells QuestNav that the robot is at {@code currentFusedPose} now
     * (converted to the headset's pose with the offset). Frames are ignored until they were
     * captured {@link #QUEST_RESET_SETTLE_SECONDS} after this, since QuestNav doesn't confirm
     * the reset. The first frame after that re-aligns the frame filter to the estimate, so the
     * delay before the headset applies the pose (no timestamp in QuestNav's setPose) leaves no
     * bias when this is called while moving.
     *
     * @param currentFusedPose The robot's field pose right now.
     */
    public void setPose(Pose2d currentFusedPose) {
        Pose3d robotPose3d = new Pose3d(currentFusedPose);
        Pose3d questPose = robotPose3d.transformBy(offset);
        questNav.setPose(questPose);
        frameFilter.reset();

        poseResetTimestamp = Timer.getFPGATimestamp();
        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("should update", () -> shouldUpdate(), null);
        builder.addBooleanProperty("is tracking", () -> isTracking, null);
        builder.addBooleanProperty("is anchored", () -> hasUpdatedQuestIntialPose, null);
        builder.addIntegerProperty("battery", () -> getBattery(), null);
    }
}