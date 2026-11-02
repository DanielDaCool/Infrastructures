package frc.robot.Path;

import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Latches and firing rules of the path events (design doc section 8.8). */
final class EventSchedule {

    /** An event s within this of a stop point s_c is a stop event [m] (5.9.5 gives s_c exactly). */
    static final double STOP_MATCH_TOL = 1e-9;

    private final ResolvedEvent[] events;
    private final boolean[] stopEvent;
    private final boolean[] fired;

    /**
     * @param events      resolved events of the active path, ascending s, same s in the order added
     * @param stopPointsS s of the stop points; events at exactly these s fire on arrival (8.6)
     */
    EventSchedule(List<ResolvedEvent> events, double[] stopPointsS) {
        this.events = sortedStable(events);
        this.stopEvent = new boolean[this.events.length];
        this.fired = new boolean[this.events.length];
        for (int i = 0; i < this.events.length; i++) {
            for (double sc : stopPointsS) {
                if (Math.abs(this.events[i].s() - sc) <= STOP_MATCH_TOL) {
                    stopEvent[i] = true;
                }
            }
        }
    }

    /** @return the events sorted by s; a stable sort keeps the order added for equal s */
    private static ResolvedEvent[] sortedStable(List<ResolvedEvent> events) {
        ResolvedEvent[] out = events.toArray(new ResolvedEvent[0]);
        java.util.Arrays.sort(out, (a, b) -> Double.compare(a.s(), b.s()));
        return out;
    }

    /** Clears every latch; called in initialize (8.14). */
    void reset() {
        java.util.Arrays.fill(fired, false);
    }

    /** Schedules every unfired non-stop event with s_event &lt;= s*, in order (8.8). */
    void update(double sStar) {
        for (int i = 0; i < events.length; i++) {
            if (!fired[i] && !stopEvent[i] && events[i].s() <= sStar) {
                fire(i);
            }
        }
    }

    /** Fires the events at the stop point sStop, on arrival before ROTATE and DWELL (8.6, 8.8). */
    void fireStopEvents(double sStop) {
        for (int i = 0; i < events.length; i++) {
            if (!fired[i] && stopEvent[i] && Math.abs(events[i].s() - sStop) <= STOP_MATCH_TOL) {
                fire(i);
            }
        }
    }

    /** Fires every unfired event on a successful finish (8.8). */
    void fireRemaining() {
        for (int i = 0; i < events.length; i++) {
            if (!fired[i]) {
                fire(i);
            }
        }
    }

    /** @return how many events have fired since the last reset (tests, telemetry) */
    int firedCount() {
        int n = 0;
        for (boolean f : fired) {
            n += f ? 1 : 0;
        }
        return n;
    }

    private void fire(int i) {
        fired[i] = true;
        CommandScheduler.getInstance().schedule(events[i].event().action());
    }

    /**
     * Throws V20 if an event command requires a subsystem the follower requires.
     *
     * @param events       resolved events
     * @param requirements the follower's requirements
     */
    static void checkRequirements(List<ResolvedEvent> events, Set<Subsystem> requirements) {
        for (ResolvedEvent e : events) {
            Command action = e.event().action();
            for (Subsystem sub : action.getRequirements()) {
                if (requirements.contains(sub)) {
                    PathEvent ev = e.event();
                    String where = ev.kind() == PathEvent.Kind.FRACTION ? "fraction " + ev.value()
                            : "point " + (int) ev.value();
                    throw new IllegalArgumentException("The event at " + where + " needs the subsystem "
                            + sub.getName() + ", which FollowPathCommand also needs. Scheduling the event would cancel"
                            + " the path. Make the event command not require the drivetrain (for example, use only the"
                            + " mechanism).");
                }
            }
        }
    }
}
