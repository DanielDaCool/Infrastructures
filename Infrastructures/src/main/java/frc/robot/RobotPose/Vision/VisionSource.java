package frc.robot.RobotPose.Vision;

import java.util.List;

/**
 * Anything that can measure the robot's field pose (a Limelight, a Quest, ...).
 * {@link frc.robot.RobotPose.RobotPose} only talks to sources through this interface.
 *
 * <p>Each loop, RobotPose calls {@link #periodic()} on every source first, and only then
 * {@link #shouldUpdate()} and {@link #getPoseEstimates()} on each one. So a source should
 * read its device in {@code periodic()} and just return what it read in the other two.
 *
 * <p>To add a new kind of source: extend {@link BaseVisionSource}, add a config class that
 * extends {@code BaseVisionSourceConfig}, and add a value for it to
 * {@code BaseVisionSourceConfig.VisionSourceType}.
 */
public interface VisionSource {
    /** @return The name from the config; also used for the dashboard paths. */
    public String getName();

    /**
     * @return Whether RobotPose should use {@link #getPoseEstimates()} this loop (e.g. the
     *         camera sees a tag). Called after {@link #periodic()}.
     */
    boolean shouldUpdate();

    /**
     * The measurements read in this loop's {@link #periodic()}. A list so a source can
     * return any number of them; empty if there is nothing new.
     *
     * <p>A frame must only be returned in one loop. Returning the same frame again would
     * make the estimator use it twice.
     */
    List<TimestampedVisionMeasurement> getPoseEstimates();

    /** @return Whether the device is currently talking to the robot (shown on the dashboard). */
    boolean isConnected();

    /**
     * Called once per loop on every source, before {@link #shouldUpdate()} or
     * {@link #getPoseEstimates()} is called on any of them. This is where the source reads
     * new data from its device (and, for MegaTag2, sends it the robot's heading first).
     */
    void periodic();
}