package frc.demacia.utils.log;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * STAND-IN for the team's frc.demacia.utils.log.Log (same package, class name and signatures for the
 * methods RobotPose uses). DELETE this file and ConsoleAlert.java when this package is moved into
 * DemaciaCode: two classes with the same name must never be on the classpath together.
 * Backed by plain WPILib: SmartDashboard for entries, WPILib Alert for alerts.
 */
public class Log {
    private static final Map<String, ConsoleAlert> conditionAlerts = new HashMap<>();

    private Log() {}

    /** Publishes a Sendable under key (the team Log sends it through its DashboardBuilder). */
    public static void putData(String key, Sendable sendable) {
        SmartDashboard.putData(key, sendable);
    }

    /** Sets the warning alert with this text; call every loop with a constant text. */
    public static synchronized ConsoleAlert alert(String message, boolean active) {
        ConsoleAlert alert = conditionAlerts.computeIfAbsent(message, ConsoleAlert::warning);
        alert.set(active);
        return alert;
    }
}
