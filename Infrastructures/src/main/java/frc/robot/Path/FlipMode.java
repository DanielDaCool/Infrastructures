package frc.robot.Path;

/** How a blue path is turned into a red one (design doc sections 5.13, 9.9). */
public enum FlipMode {
    /** No flip; the team writes separate red paths. */
    NONE,
    /** Mirror about x = fieldLength / 2. */
    MIRROR,
    /** Rotate 180 degrees about the field center. */
    ROTATE
}
