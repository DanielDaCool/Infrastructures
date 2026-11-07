package frc.robot.Path;

/** Tuning for the Path module. Units: m, s, rad. Change per robot / season. */
public final class PathConstants {
    private PathConstants() {}

    /** Default corner radius [m]. */
    public static final double RADIUS = 0.25;
    public static final double MAX_VELOCITY = 2.0; // m/s
    public static final double MAX_ACCELERATION = 3.0; // m/s^2, along the path
    public static final double MAX_DECELERATION = 3.0; // m/s^2, along the path
    public static final double MAX_LATERAL_ACCELERATION = 3.0; // m/s^2, caps arc speed
    public static final double MAX_VECTOR_ACCELERATION = 6.0; // m/s^2, change of the velocity vector
    public static final double MAX_OMEGA = Math.PI * 2; // rad/s
    public static final double HEADING_KP = 4.0; // (rad/s) per rad
    /** Gain on the remaining distance to a stop and on the cross-track error [1/s]. */
    public static final double FINAL_KP = 3.0;
    /** Cross-track distance at which the merge angle is 45 deg [m]. */
    public static final double MERGE_DISTANCE = 0.5;
    /** Drivetrain lag the direction lookahead compensates [s]. */
    public static final double LAG_TIME = 0.05;
    public static final double POSITION_TOLERANCE = 0.03; // m
    public static final double ANGLE_TOLERANCE = Math.toRadians(2);
    /** Measured speed below which a stop counts as reached [m/s]. */
    public static final double STOP_SPEED = 0.1;
    /** Max distance from the robot to the path start [m]. */
    public static final double START_TOLERANCE = 0.5;
    /** Corners turning more than this become stops. */
    public static final double MAX_ARC_TURN = Math.toRadians(150);
    public static final double DT = 0.02; // s, loop period
    public static final double FIELD_LENGTH = 16.54; // m
    public static final double FIELD_WIDTH = 8.07; // m
    /** false: red = 180 deg rotation about the field center; true: mirror across x = FIELD_LENGTH/2. */
    public static final boolean MIRRORED_FIELD = false;
}
