package frc.robot.Path;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Where Path sends its dashboard entries and alerts. By default they go to SmartDashboard and
 * WPILib {@link Alert}s ({@link #WPILIB}). To use your team's logger, implement this interface
 * and set it once in {@code Robot}'s constructor, before any {@code Path} is created:
 *
 * <pre>{@code
 * PathLogger.set(new PathLogger() {
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
public interface PathLogger {

    /**
     * Publishes a dashboard entry, like {@code SmartDashboard.putData}. Called once per key.
     *
     * @param key      entry name, e.g. {@code "Path/<name>"}
     * @param sendable what to publish
     */
    void putData(String key, Sendable sendable);

    /**
     * Shows or clears an alert.
     *
     * @param text   alert text, constant for one alert
     * @param type   {@link AlertType#kError} or {@link AlertType#kWarning}
     * @param active true while the problem lasts
     */
    void alert(String text, AlertType type, boolean active);

    /** The default: SmartDashboard for entries, one WPILib {@link Alert} per text and type. */
    PathLogger WPILIB = new PathLogger() {
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
     * Sets the logger Path uses.
     *
     * @param logger the team's logger, or {@link #WPILIB}
     */
    static void set(PathLogger logger) {
        Holder.logger = Objects.requireNonNull(logger, Point.nullMessage("PathLogger.set", "the logger",
                "a PathLogger (for example PathLogger.WPILIB)"));
    }

    /** @return the logger Path uses ({@link #WPILIB} until {@link #set} is called) */
    static PathLogger get() {
        return Holder.logger;
    }

    /** Holds the current logger (interfaces can't have mutable static fields). */
    final class Holder {
        private static volatile PathLogger logger = WPILIB;

        private Holder() {
        }
    }
}
