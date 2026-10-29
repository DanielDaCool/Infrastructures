package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.function.DoubleFunction;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Decides which Quest frames are used and moves them onto the fused pose (used by
 * {@link Quest}). The Quest's own pose can jump (after it re-localizes or regains tracking)
 * and its frames come at ~100 Hz with errors that are the same from one frame to the next, so:
 * <ul>
 * <li><b>Jump or tracking regained:</b> if the Quest moved more than {@code jumpMeters}
 * differently from the odometry between two frames, or no frame came for {@code gapSeconds},
 * its frames are ignored for {@code blackoutSeconds}, then it is re-aligned: the offset
 * between its position and the estimate at that frame is stored and added to every later
 * frame.</li>
 * <li><b>New anchor:</b> after {@link #reset} (the Quest was set to a new pose) the first
 * frame re-aligns it the same way. If the headset applies the pose only after that frame,
 * the pose change reads as a jump and is handled as one.</li>
 * <li><b>Thinning:</b> at most one frame per {@code periodSeconds} is used (the first one of
 * each period), with a larger std dev (see {@code QuestConfig}).</li>
 * </ul>
 * In simulation this took the error during a Quest jump from 49 cm to 6 cm, and without
 * thinning the Quest outvoted the cameras (26-28 runs lost for more than 5 s).
 */
public final class QuestFrameFilter {
    /** Odometry motion between two FPGA times, field frame. */
    public interface OdometryMotion {
        Translation2d between(double fromSeconds, double toSeconds);
    }

    private final double jumpMeters;
    private final double gapSeconds;
    private final double blackoutSeconds;
    private final double periodSeconds;

    /** Time and raw position of the previous frame; NaN/null before the first. */
    private double lastFrameSeconds = Double.NaN;
    private Translation2d lastFramePosition;
    /** Frames before this time are ignored (after a jump). */
    private double ignoreUntilSeconds = Double.NEGATIVE_INFINITY;
    private boolean realignNext = false;
    /** Added to the Quest position to get the robot position on the fused field. */
    private Translation2d offset = Translation2d.kZero;
    /** Time of the last used (or re-aligning) frame, for the thinning. */
    private double lastUsedSeconds = Double.NEGATIVE_INFINITY;

    /**
     * @param jumpMeters      Quest motion minus odometry motion between two frames that counts as a jump.
     * @param gapSeconds      Time without frames that counts as tracking lost.
     * @param blackoutSeconds How long frames are ignored after a jump.
     * @param periodSeconds   At most one frame per this long is used.
     */
    public QuestFrameFilter(double jumpMeters, double gapSeconds, double blackoutSeconds, double periodSeconds) {
        this.jumpMeters = jumpMeters;
        this.gapSeconds = gapSeconds;
        this.blackoutSeconds = blackoutSeconds;
        this.periodSeconds = periodSeconds;
    }

    /**
     * @param timestampSeconds FPGA time the frame was captured.
     * @param questPosition    The robot position the Quest measured (its own field).
     * @param estimateAt       The fused position at an FPGA time.
     * @param odometryMotion   The odometry motion between two FPGA times.
     * @return The robot position to use (Quest position plus the offset), or null if this
     *         frame isn't used.
     */
    public Translation2d filter(double timestampSeconds, Translation2d questPosition,
            DoubleFunction<Translation2d> estimateAt, OdometryMotion odometryMotion) {
        if (lastFramePosition != null) {
            Translation2d odometryDelta = odometryMotion.between(lastFrameSeconds, timestampSeconds);
            boolean jumped = questPosition.minus(lastFramePosition).minus(odometryDelta).getNorm() > jumpMeters;
            if (jumped || timestampSeconds - lastFrameSeconds > gapSeconds) {
                ignoreUntilSeconds = timestampSeconds + blackoutSeconds;
                realignNext = true;
            }
        }
        lastFrameSeconds = timestampSeconds;
        lastFramePosition = questPosition;
        if (timestampSeconds < ignoreUntilSeconds) {
            return null;
        }
        if (realignNext) {
            offset = estimateAt.apply(timestampSeconds).minus(questPosition);
            realignNext = false;
            lastUsedSeconds = timestampSeconds;
            return null;
        }
        if (timestampSeconds - lastUsedSeconds < periodSeconds - 1e-9) {
            return null;
        }
        lastUsedSeconds = timestampSeconds;
        return questPosition.plus(offset);
    }

    /**
     * Moves the offset by {@code shift}: the estimator found that the Quest frames agree with
     * each other but not with the estimate (see {@code DemaciaPoseEstimator.VisionResult}).
     */
    public void shiftOffset(Translation2d shift) {
        offset = offset.plus(shift);
    }

    /** @return What is added to the Quest position (meters, field frame). */
    public Translation2d getOffset() {
        return offset;
    }

    /**
     * Forgets everything (after the Quest was set to a new pose) and re-aligns on the next
     * frame. The headset applies setPose a little later than it was sent, while the robot
     * moves, so the anchor alone is off by speed x that delay.
     */
    public void reset() {
        lastFrameSeconds = Double.NaN;
        lastFramePosition = null;
        ignoreUntilSeconds = Double.NEGATIVE_INFINITY;
        realignNext = true;
        offset = Translation2d.kZero;
        lastUsedSeconds = Double.NEGATIVE_INFINITY;
    }
}
