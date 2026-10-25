package frc.robot.RobotPose;

import java.util.Objects;
import java.util.function.BiConsumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Where RobotPose sends its dashboard entries. Every entry is a {@link Sendable} under a key
 * (e.g. {@code "pose"}, {@code "vision/front"}). SmartDashboard by default; to use the team's
 * logger, call this once in {@code Robot}'s constructor, before {@code RobotPose.initialize}:
 *
 * <pre>{@code
 * PoseLog.setLogger(Log::putData); // frc.demacia.utils.log.Log
 * }</pre>
 */
public final class PoseLog {

    private static BiConsumer<String, Sendable> logger = SmartDashboard::putData;

    private PoseLog() {
    }

    /**
     * @param logger takes a key and a {@link Sendable}, like {@code SmartDashboard.putData}.
     *               Entries already sent stay where they were; call this before
     *               {@code RobotPose.initialize}.
     */
    public static synchronized void setLogger(BiConsumer<String, Sendable> logger) {
        PoseLog.logger = Objects.requireNonNull(logger);
    }

    /** Sends one entry to the logger. */
    public static synchronized void putData(String key, Sendable sendable) {
        logger.accept(key, sendable);
    }
}
