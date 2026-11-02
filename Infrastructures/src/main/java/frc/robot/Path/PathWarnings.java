package frc.robot.Path;

import java.util.List;

import edu.wpi.first.wpilibj.Alert.AlertType;

/** Collects warnings and sends them through {@link PathLogger#alert} (design doc sections 9.12, 7.13.7). */
final class PathWarnings {

    /** Jerk warning threshold is max(aAcc, aDec) / (5 * dt_nom) (section 10, C2). */
    static final double JERK_WARNING_DT_FACTOR = 5.0;

    /** @param pathName the path name used in the message prefix {@code Path "name"} */
    PathWarnings(String pathName) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /**
     * Build warning W1 to W5: formats {@code Path "name", point i: text}, sends it with
     * kWarning/active=true once, stores it (9.12.2).
     *
     * @param pointIndex user point index, or -1 for a warning without a point
     * @param text       the text of the table row
     */
    void add(int pointIndex, String text) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return the warning strings collected so far (backs {@code Path.warnings()}) */
    List<String> list() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** Sends a start warning (C1, C2) once per text (9.12.3). */
    static void startWarning(String text) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** Sets or clears a run-time alert A1 to A3 (9.12.3). */
    static void runtimeAlert(String text, AlertType type, boolean active) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return max(aAcc, aDec) / (5 * dtNom): the jerk above which C2 is raised (7.13.7) */
    static double jerkWarningThreshold(double aAcc, double aDec, double dtNom) {
        return Math.max(aAcc, aDec) / (JERK_WARNING_DT_FACTOR * dtNom);
    }
}
