package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
 * <p>Only the newest unread frame each loop is used, and only if it is tracking. Its
 * position is converted from the headset to the robot center with the config offset; the
 * heading is copied from the estimate at the frame's time, so only x/y are measured (theta
 * is sent with an infinite std dev, like the Limelights).
 */
public class Quest extends BaseVisionSource {
    private final QuestNav questNav;

    /** Capture time of {@link #pose} (QuestNav's data timestamp, FPGA time). */
    private double timestampSeconds;
    private PoseFrame[] poseFrames;

    /** True once frames from after the last anchor are arriving; see class docs. */
    private boolean hasUpdatedQuestIntialPose;
    /** True at startup and after a disconnect, until re-anchored; see class docs. */
    private boolean hasQuestDisconnected;
    /** Whether this loop's periodic() got a new tracking frame. */
    private boolean hasNewPose;
    /** FPGA time of the last {@link #setPose}; infinite until the first one. */
    private double poseResetTimestamp = Double.POSITIVE_INFINITY;

    /**
     * QuestNav does not acknowledge setPose(), so frames are only trusted once they were
     * captured this long after the reset was sent.
     */
    private static final double QUEST_RESET_SETTLE_SECONDS = 0.25;

    /**
     * @param config The Quest's config. offset is robot center to headset:
     *               {@code robotPose = questPose.transformBy(offset.inverse())} and
     *               {@code questPose = robotPose.transformBy(offset)}. The name is only used
     *               for the dashboard.
     */
    public Quest(QuestConfig config) {
        super(config);
        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = true;
        questNav = new QuestNav();
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
        SmartDashboard.putData("vision/" + getName() + "/Reset Quest Pose", new InstantCommand(
            ()->setPose(RobotPose.getInstance().getEstimatedPose())).ignoringDisable(true));
    }

    /**
     * @return True if the Quest is connected, has been anchored and settled, and this loop
     *         got a new tracking frame.
     */
    @Override
    public boolean shouldUpdate() {
        return isConnected() && hasUpdatedQuestIntialPose && hasNewPose;
    }

    /**
     * @return The measurement from this loop's newest frame, with the config x/y std devs.
     *         The theta std dev is infinite because the heading was copied from the
     *         estimate, not measured. Only valid when {@link #shouldUpdate()} is true.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        return List.of(new TimestampedVisionMeasurement(pose, timestampSeconds, 
                VecBuilder.fill(std.get(0, 0), std.get(1, 0), Double.POSITIVE_INFINITY)));
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
     * Runs QuestNav's own update (it must run every loop), reads all unread frames, and
     * builds {@link #pose} from the newest one if it is tracking. Also updates the anchor
     * state: marks the Quest settled once a frame from after the settle time arrives, and
     * marks it disconnected (so it gets re-anchored) if the connection drops.
     */
    @Override
    public void periodic() {
        super.periodic();
        
        questNav.commandPeriodic();

        poseFrames = questNav.getAllUnreadPoseFrames();
        hasNewPose = false;

        if (poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()) {
            timestampSeconds = poseFrames[poseFrames.length - 1].dataTimestamp();

            pose = new Pose2d(poseFrames[poseFrames.length - 1].questPose3d()
                .transformBy(offset.inverse()).toPose2d().getTranslation(), 
                RobotPose.getInstance().getEstimatedPoseAt(timestampSeconds).getRotation());

            hasNewPose = true;

            if (!hasQuestDisconnected && timestampSeconds > poseResetTimestamp + QUEST_RESET_SETTLE_SECONDS) {
                hasUpdatedQuestIntialPose = true;
            }
        }

        if (!hasQuestDisconnected && !isConnected()) {
            hasQuestDisconnected = true;
            hasUpdatedQuestIntialPose = false;
        }
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
     * the reset.
     *
     * @param currentFusedPose The robot's field pose right now.
     */
    public void setPose(Pose2d currentFusedPose) {
        Pose3d robotPose3d = new Pose3d(currentFusedPose);
        Pose3d questPose = robotPose3d.transformBy(offset);
        questNav.setPose(questPose);

        poseResetTimestamp = Timer.getFPGATimestamp();
        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("should update", () -> shouldUpdate(), null);
        builder.addIntegerProperty("battery", () -> getBattery(), null);
    }
}