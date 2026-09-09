package frc.robot.RobotPose.Vision;

import java.util.List;

/**
 * Uniform contract every vision-producing device implements
 *
 */
public interface VisionSource {

    /**
     * Per-source, source-native gate deciding whether it's worth asking this source for
     * data this loop at all. See class docs -- semantics differ by implementation.
     */
    boolean shouldUpdate();

    /**
     * Always returns a List, even for sources that only ever produce one measurement per
     * loop -- this uniformity is what lets RobotPose treat "1 source, 1 measurement" and
     * "1 source (Quest), up to several measurements" identically with no special-casing.
     * Returns an empty list if this source has nothing to report this loop.
     */
    List<TimestampedVisionMeasurement> getPoseEstimates();

    /** Lets RobotPose (or a caller) surface camera/device dropout, e.g. to a dashboard. */
    boolean isConnected();

    /**
     * Called once per loop on every registered source, regardless of shouldUpdate()'s
     * result -- this is where a source does per-loop device-specific work that must happen
     * before its own (or, per RobotPose's two-pass design, any source's) pose can be
     * correctly read this same loop. Example: Limelight 3D/MegaTag2 pushing gyro heading
     * via SetRobotOrientation() before reading pose back.
     */
    void periodic();
}