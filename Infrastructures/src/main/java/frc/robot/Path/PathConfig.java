package frc.robot.Path;

/** Values to set for your robot and season. Units: m, s, rad. */
public final class PathConfig {
    private PathConfig() {}

    /** Top speed along the path [m/s]. */
    public static final double MAX_VELOCITY = 2.0;
    /** Acceleration along the path; also used for braking, arc speed and (doubled) velocity change [m/s^2]. */
    public static final double MAX_ACCELERATION = 3.0;
    /** Top turn rate [rad/s]. */
    public static final double MAX_OMEGA = Math.PI * 2;
    /** Default corner radius [m]. */
    public static final double RADIUS = 0.25;
    /** Distance to a stop or the end that counts as reached [m]. */
    public static final double POSITION_TOLERANCE = 0.03;
    /** Heading error at the end that counts as reached [rad]. */
    public static final double ANGLE_TOLERANCE = Math.toRadians(2);
    /** true: red is mirrored across the field center line; false: rotated 180 deg. Set per season. */
    public static final boolean MIRRORED_FIELD = false;
}
