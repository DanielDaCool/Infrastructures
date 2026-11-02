package frc.robot.Path;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * An action at a fraction of the path or at a point index (design doc sections 8.8, 9.7).
 * Immutable. The fraction or index is checked (V19) when the event is added to a {@code Path}.
 */
public final class PathEvent {

    private static final String ACTION_HINT = "a Command or a Runnable (for example intake.runCommand())";

    /** How {@link #value()} is read. */
    public enum Kind { FRACTION, POINT_INDEX }

    private final Kind kind;
    private final double value;
    private final Command action;

    private PathEvent(Kind kind, double value, Command action) {
        this.kind = kind;
        this.value = value;
        this.action = Objects.requireNonNull(action, Point.nullMessage("PathEvent", "the action", ACTION_HINT));
    }

    /** @return an event at fraction f of the path length (0 = start, 1 = end) */
    public static PathEvent atFraction(double fraction, Command action) {
        return new PathEvent(Kind.FRACTION, fraction, action);
    }

    /** @return an event at fraction f; the Runnable is wrapped with {@code Commands.runOnce} */
    public static PathEvent atFraction(double fraction, Runnable action) {
        Objects.requireNonNull(action, Point.nullMessage("PathEvent.atFraction", "the action", ACTION_HINT));
        return atFraction(fraction, Commands.runOnce(action));
    }

    /** @return an event at the point the user wrote at pointIndex */
    public static PathEvent atPoint(int pointIndex, Command action) {
        return new PathEvent(Kind.POINT_INDEX, pointIndex, action);
    }

    /** @return an event at a point index; the Runnable is wrapped with {@code Commands.runOnce} */
    public static PathEvent atPoint(int pointIndex, Runnable action) {
        Objects.requireNonNull(action, Point.nullMessage("PathEvent.atPoint", "the action", ACTION_HINT));
        return atPoint(pointIndex, Commands.runOnce(action));
    }

    /** @return FRACTION or POINT_INDEX */
    public Kind kind() {
        return kind;
    }

    /** @return the fraction or the index, as given */
    public double value() {
        return value;
    }

    /** @return the command to schedule */
    public Command action() {
        return action;
    }
}
