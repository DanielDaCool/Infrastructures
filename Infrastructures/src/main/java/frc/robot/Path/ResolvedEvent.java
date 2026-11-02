package frc.robot.Path;

/**
 * A {@link PathEvent} converted to arc length when it was added to a {@code Path} (design doc
 * sections 5.9.5, 9.7).
 *
 * @param s     arc length of the event [m], in [0, L]
 * @param event the event as the user gave it
 */
public record ResolvedEvent(double s, PathEvent event) {
}
