package frc.robot.Path;

import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Latches and firing rules of the path events (design doc section 8.8). */
final class EventSchedule {

    /**
     * @param events      resolved events of the active path, ascending s, same s in the order added
     * @param stopPointsS s of the stop points; events at exactly these s fire on arrival (8.6)
     */
    EventSchedule(List<ResolvedEvent> events, double[] stopPointsS) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** Clears every latch; called in initialize (8.14). */
    void reset() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** Schedules every unfired non-stop event with s_event &lt;= s*, in order (8.8). */
    void update(double sStar) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** Fires the events at the stop point sStop, on arrival before ROTATE and DWELL (8.6, 8.8). */
    void fireStopEvents(double sStop) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** Fires every unfired event on a successful finish (8.8). */
    void fireRemaining() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /**
     * Throws V20 if an event command requires a subsystem the follower requires.
     *
     * @param events       resolved events
     * @param requirements the follower's requirements
     */
    static void checkRequirements(List<ResolvedEvent> events, Set<Subsystem> requirements) {
        throw new UnsupportedOperationException("TODO phase 3");
    }
}
