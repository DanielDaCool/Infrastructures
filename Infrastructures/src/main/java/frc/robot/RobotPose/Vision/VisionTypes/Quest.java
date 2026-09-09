package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionSourceConfig;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Wraps a QuestNav-connected Meta Quest headset as a VisionSource, plus the two additional
 * methods (hasDrifted/updatePose) RobotPose's re-anchor logic depends on. Uses QuestNav's
 * real, confirmed API (https://questnav.gg) -- commandPeriodic(), isConnected(),
 * isTracking(), getAllUnreadPoseFrames(), setPose(Pose3d) -- verified against QuestNav's own
 * current documentation rather than assumed.
 *
 * shouldUpdate()/isConnected()/periodic()/hasDrifted()/updatePose() are fully implemented
 * below. getPoseEstimates() is intentionally left as a stub -- per instruction, the actual
 * frame-to-measurement math (turning drained PoseFrames into TimestampedVisionMeasurements,
 * including the robotToQuest offset correction and per-frame stdDevs) is the user's to
 * write, not this assistant's.
 */
public class Quest implements VisionSource {

    private final QuestNav questNav;
    private final Transform3d robotToQuest;

    /**
     * Per-frame tracking-transition detection for hasDrifted(). Tracks whether the LAST
     * frame processed (across all calls, not just within one loop) was tracking, so a
     * false->true transition that occurs mid-batch (multiple frames drained in a single
     * getAllUnreadPoseFrames() call) is still correctly detected -- per the confirmed
     * design (isTracking is a PER-FRAME field, not a device-level flag, and a transition
     * can occur entirely within one loop's batch of frames).
     */
    private boolean wasTrackingLastFrame = false;
    private boolean driftedSinceLastCheck = false;

    /**
     * @param config Static configuration for this source. offset is robotToQuest -- the
     *               geometric transform from the robot's center to the Quest headset's
     *               mount position, used to convert between the Quest's own reported pose
     *               and the robot's pose. (Confirmed real usage pattern from QuestNav's own
     *               docs: robotPose = questPose.transformBy(robotToQuest.inverse()) and
     *               questPose = robotPose.transformBy(robotToQuest) for the reverse
     *               direction, used by updatePose() below.) sourceName is accepted as part
     *               of the shared VisionSourceConfig shape but is not otherwise used by
     *               this class -- QuestNav does not use a name-based lookup the way
     *               Limelight does.
     */
    public Quest(VisionSourceConfig config) {
        this.questNav = new QuestNav();
        this.robotToQuest = config.offset();
    }

    /**
     * Gated on QuestNav's own isConnected() -- a coarser check than per-frame tracking
     * (isTracking), matching the confirmed design: shouldUpdate() answers "is it even worth
     * asking this loop," with finer-grained per-frame tracking filtering happening
     * separately, inside getPoseEstimates().
     */
    @Override
    public boolean shouldUpdate() {
        return questNav.isConnected();
    }

    /**
     *
     * @return A list of pose measurements from this loop's unread frames, filtered to only
     *         those that were actually tracking.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        throw new UnsupportedOperationException(
                "getPoseEstimates() is not yet implemented -- fill in the frame-processing math here");
    }

    @Override
    public boolean isConnected() {
        return questNav.isConnected();
    }

    /** Required by QuestNav to process incoming/outgoing data -- must be called every loop. */
    @Override
    public void periodic() {
        questNav.commandPeriodic();
    }

    /**
     * Detects a false->true isTracking transition, INCLUDING one that occurred entirely
     * within a single batch of frames drained by getPoseEstimates() (not just across
     * separate periodic() calls) -- per the confirmed design, this is why the decision
     * logic has to live inside Quest itself: only code with access to the raw per-frame
     * sequence (which getPoseEstimates(), once implemented, will drain and discard) can
     * correctly detect a transition that happens mid-batch.
     *
     * NOTE: this method's correctness currently depends on getPoseEstimates() (the stub
     * above) actually updating wasTrackingLastFrame/driftedSinceLastCheck as it processes
     * each frame -- until that's implemented, this will always report false (no frames are
     * ever processed to detect a transition from). This is flagged rather than hidden: the
     * FIELDS and the QUERY method are both real and complete, but the field-updating logic
     * only becomes real once getPoseEstimates() is implemented.
     *
     * Calling this consumes the drift flag (resets it to false) -- matching the confirmed
     * two-method contract (hasDrifted() is a query, updatePose() is the action RobotPose
     * takes in response), so a single detected drift event triggers exactly one re-anchor,
     * not a repeated one every loop until something else resets it.
     */
    public boolean hasDrifted() {
        boolean result = driftedSinceLastCheck;
        driftedSinceLastCheck = false;
        return result;
    }

    /**
     * Pushes a corrected pose to the Quest, converting from the robot's field pose to the
     * Quest's own pose via the same robotToQuest transform used for reading (confirmed real
     * pattern from QuestNav's docs: questPose = robotPose.transformBy(robotToQuest)), then
     * calls questNav.setPose(Pose3d) -- QuestNav's own confirmed, real re-anchor method.
     *
     * Called by RobotPose only when hasDrifted() is true (per the confirmed two-method
     * contract) -- this method itself does not check hasDrifted() or any other condition,
     * it unconditionally does what it's told, matching the "action, not decision" half of
     * the contract.
     *
     * @param currentFusedPose The robot's current best-estimate field pose (from
     *                          RobotPose.getEstimatedPose()) to re-anchor the Quest against.
     */
    public void updatePose(Pose2d currentFusedPose) {
        Pose3d robotPose3d = new Pose3d(currentFusedPose);
        Pose3d questPose = robotPose3d.transformBy(robotToQuest);
        questNav.setPose(questPose);
    }
}