package frc.robot.RobotPose.Vision.VisionTypes;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotPose.Vision.LimelightHelpers;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionSourceConfig;

/**
 * Wraps a single Limelight camera in 3D (MegaTag2) mode as a VisionSource. 
 *
 */
public class LimelightTagCamera3d implements VisionSource {

    private final String sourceName;

    /**
     * Supplies the current gyro heading that periodic() pushes to the Limelight every loop
     * via SetRobotOrientation() -- MegaTag2 REQUIRES this to happen before this same loop's
     * pose can be correctly read back (confirmed hard requirement, not optional). This is a
     * live wiring dependency, not static configuration, so it stays its own constructor
     * parameter rather than living inside VisionSourceConfig.
     */
    private final Supplier<Rotation2d> gyroHeadingSupplier;

    // --- Connectivity tracking (see isConnected() below) --------------------------------
    private double lastFrameCounterValue = -1.0;
    private double lastFrameCounterChangeTime = 0.0;

    /**
     * How long the Limelight's frame counter (LimelightHelpers.getHeartbeat()) may go
     * without incrementing before this camera is considered disconnected. FLAGGED: this
     * threshold is this assistant's own reasonable default, not a value confirmed against
     * real hardware behavior or Limelight's own documentation -- reconsider/tune if it
     * doesn't match observed behavior (e.g. false disconnects during normal pipeline
     * switches).
     */
    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 1.0;

    /**
     * @param config              Static configuration for this source. sourceName is the
     *                            Limelight's NetworkTables name. offset is currently unused
     *                            by this class (Limelight's own pose math accounts for
     *                            camera placement via its own calibration) but is accepted
     *                            here as part of the shared VisionSourceConfig shape used
     *                            by every VisionSource.
     * @param gyroHeadingSupplier Supplies the robot's current field-relative gyro heading.
     */
    public LimelightTagCamera3d(VisionSourceConfig config, Supplier<Rotation2d> gyroHeadingSupplier) {
        this.sourceName = config.sourceName();
        this.gyroHeadingSupplier = gyroHeadingSupplier;
    }

    /**
     * Gated on LimelightHelpers.getTV(), matching the tv >= 0.1 pattern from the user's own
     * old TagPose.isSeeTag() (explicitly confirmed as the model for this method) --
     * getTV() is LimelightHelpers' own boolean wrapper around that same NetworkTables
     * entry, so this is the same gate, just via the standard helper rather than a raw read.
     */
    @Override
    public boolean shouldUpdate() {
        return LimelightHelpers.getTV(sourceName);
    }

    /**
     *
     * @return A list of pose measurements (one per visible tag) from this loop, or an empty
     *         list if none should be reported.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
       return null; //TODO finish the math
    }

    /**
     * Limelight cameras communicate over NetworkTables; LimelightHelpers does not expose a
     * direct "is this Limelight physically connected" boolean, so connectivity is inferred
     * from LimelightHelpers.getHeartbeat(sourceName) -- a counter that increments once per
     * frame while the camera is alive (confirmed real API). Since it's a raw counter, not a
     * boolean, connectivity has to be inferred by checking whether it's still CHANGING over
     * time, not just reading it once -- so this tracks the last-seen value and when it last
     * changed, and reports disconnected only if the counter has been stuck for longer than
     * CAMERA_STALE_TIMEOUT_SECONDS.
     */
    @Override
    public boolean isConnected() {
        double currentFrameCounter = LimelightHelpers.getHeartbeat(sourceName);
        double now = Timer.getFPGATimestamp();

        if (currentFrameCounter != lastFrameCounterValue) {
            lastFrameCounterValue = currentFrameCounter;
            lastFrameCounterChangeTime = now;
        }

        return (now - lastFrameCounterChangeTime) < CAMERA_STALE_TIMEOUT_SECONDS;
    }

    /**
     * Pushes the current gyro heading via SetRobotOrientation() BEFORE this loop's pose can
     * be correctly read back -- confirmed hard MegaTag2 requirement, not optional. This is
     * exactly why RobotPose calls periodic() on every VisionSource before calling
     * shouldUpdate()/getPoseEstimates() on any of them (the two-pass design): this call
     * must happen before getPoseEstimates() runs for THIS SAME LOOP, and the two-pass
     * structure guarantees that ordering regardless of source iteration order.
     */
    @Override
    public void periodic() {
        Rotation2d heading = gyroHeadingSupplier.get();
        LimelightHelpers.SetRobotOrientation(sourceName, heading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}