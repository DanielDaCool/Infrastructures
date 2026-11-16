package frc.robot.Path.constants;

public class PathConstants {
    public static final double MAX_VELOCITY = 4; // m/s
    public static final double MAX_ACCELERATION = 3; // m/s^2
    public static final double MAX_LATERAL_ACCELERATION = 3; // m/s^2
    public static final double MAX_ANGULAR_VELOCITY = 2;//Math.PI; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = 0.6;//Math.PI * 2; // rad/s^2
    public static final double HEADING_KP = 1; // (rad/s)/rad
    public static final double HEADING_TOLERANCE = Math.toRadians(5); // rad
    public static final double MIN_TURN_FOR_ARC = Math.toRadians(1); // rad
    public static final double MAX_ARC_SWEEP = Math.toRadians(270); // rad
    public static final double MAX_RADIUS_TO_LEG_RATIO = 0.25; // arc radius <= this * shorter leg, so close corners don't overlap
    public static final double MAX_VELOCITY_TO_CONSIDER_STOPPED = 0.1; // m/s
    public static final double MAX_POSITION_THRESHOLD_FINAL_POINT = 0.1; // m
    public static final double MAX_POSITION_THRESHOLD_DURING_PATH = 0.1; // m
    public static final double MAX_ANGLE_TO_PASS = Math.toRadians(60); // rad
    //TODO: CHANGE THIS
}
