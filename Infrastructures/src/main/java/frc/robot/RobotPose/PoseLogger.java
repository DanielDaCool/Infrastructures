package frc.robot.RobotPose;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Where RobotPose sends its dashboard entries and alerts. By default they go to SmartDashboard
 * and WPILib {@link Alert}s ({@link #WPILIB}). To use your team's logger, implement this
 * interface and set it once in {@code Robot}'s constructor, before {@code RobotPose.initialize}
 * and before the vision sources are created:
 *
 * <pre>{@code
 * PoseLogger.set(new PoseLogger() {
 *     public void putData(String key, Sendable sendable) {
 *         // send it with your logger
 *     }
 *
 *     public void alert(String text, AlertType type, boolean active) {
 *         // show or clear the alert with your logger
 *     }
 * });
 * }</pre>
 *
 * Entries sent before {@link #set} stay where they were.
 */
public interface PoseLogger {

    /**
     * Publishes a dashboard entry, like {@code SmartDashboard.putData}. Called once per key, when
     * the object that owns the entry is created.
     *
     * @param key      entry name, e.g. {@code "pose"} or {@code "vision/front"}
     * @param sendable what to publish (RobotPose, a vision source, a Field2d, a command)
     */
    void putData(String key, Sendable sendable);

    /**
     * Shows or clears an alert. Called every loop with the same text for the same alert; may be
     * called from the odometry thread's caller, not only the main loop.
     *
     * @param text   alert text, constant for one alert
     * @param type   {@link AlertType#kError} or {@link AlertType#kWarning}
     * @param active true while the problem lasts
     */
    void alert(String text, AlertType type, boolean active);

    /** The default: SmartDashboard for entries, one WPILib {@link Alert} per text and type. */
    PoseLogger WPILIB = new PoseLogger() {
        private final Map<String, Alert> alerts = new HashMap<>();

        @Override
        public void putData(String key, Sendable sendable) {
            SmartDashboard.putData(key, sendable);
        }

        @Override
        public synchronized void alert(String text, AlertType type, boolean active) {
            alerts.computeIfAbsent(type + ":" + text, k -> new Alert(text, type)).set(active);
        }
    };

    /**
     * Sets the logger RobotPose uses.
     *
     * @param logger the team's logger, or {@link #WPILIB}
     */
    static void set(PoseLogger logger) {
        Holder.logger = Objects.requireNonNull(logger);
    }

    /** @return the logger RobotPose uses ({@link #WPILIB} until {@link #set} is called) */
    static PoseLogger get() {
        return Holder.logger;
    }

    /** Holds the current logger (interfaces can't have mutable static fields). */
    final class Holder {
        private static volatile PoseLogger logger = WPILIB;

        private Holder() {
        }
    }
}
