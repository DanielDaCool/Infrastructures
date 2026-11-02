package frc.robot.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.Alert.AlertType;

/** Collects warnings and sends them through {@link PathLogger#alert} (design doc sections 9.12, 7.13.7). */
final class PathWarnings {

    /** Jerk warning threshold is max(aAcc, aDec) / (5 * dt_nom) (section 10, C2). */
    static final double JERK_WARNING_DT_FACTOR = 5.0;

    /** Start-warning texts already sent (C1, C2 are reported once per text). */
    private static final Set<String> SENT_START_WARNINGS = new HashSet<>();

    private final String pathName;
    private final List<String> warnings = new ArrayList<>();

    /** @param pathName the path name used in the message prefix {@code Path "name"} */
    PathWarnings(String pathName) {
        this.pathName = pathName;
    }

    /**
     * Build warning W1 to W5: formats {@code Path "name", point i: text}, sends it with
     * kWarning/active=true once, stores it (9.12.2).
     *
     * @param pointIndex user point index, or -1 for a warning without a point
     * @param text       the text of the table row
     */
    void add(int pointIndex, String text) {
        String message = "Path \"" + pathName + "\"" + (pointIndex >= 0 ? ", point " + pointIndex : "") + ": " + text;
        if (warnings.contains(message)) {
            return;
        }
        warnings.add(message);
        PathLogger.get().alert(message, AlertType.kWarning, true);
    }

    /** @return the warning strings collected so far (backs {@code Path.warnings()}) */
    List<String> list() {
        return Collections.unmodifiableList(new ArrayList<>(warnings));
    }

    /** Sends a start warning (C1, C2) once per text (9.12.3). */
    static void startWarning(String text) {
        synchronized (SENT_START_WARNINGS) {
            if (!SENT_START_WARNINGS.add(text)) {
                return;
            }
        }
        PathLogger.get().alert(text, AlertType.kWarning, true);
    }

    /** Sets or clears a run-time alert A1 to A3 (9.12.3). */
    static void runtimeAlert(String text, AlertType type, boolean active) {
        PathLogger.get().alert(text, type, active);
    }

    /** @return max(aAcc, aDec) / (5 * dtNom): the jerk above which C2 is raised (7.13.7) */
    static double jerkWarningThreshold(double aAcc, double aDec, double dtNom) {
        return Math.max(aAcc, aDec) / (JERK_WARNING_DT_FACTOR * dtNom);
    }
}
