// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

/** Add your docs here. */
public class KinematicsConstants {

    public static final double MAX_ALLOWED_MODULE_VELOCITY = 4.5;
    public static final double CYCLE_DT = 0.02;
    
    public static final double MIN_VELOCITY = 0.01; // slower is 0
    
    public static final double MAX_RADIAL_ACCEL = 10;
    public static final double MAX_LINEAR_ACCEL = 22; 
    public static final double MAX_DELTA_V = MAX_LINEAR_ACCEL * CYCLE_DT;
    public static final double MAX_FAST_TURN_ANGLE = MAX_RADIAL_ACCEL / MAX_LINEAR_ACCEL; //if angle is lower, no need to slow down to turn (max angle the robot can keep it's speeds while rotating)
    public static final double MIN_REVERSE_ANGLE = Math.PI - MAX_FAST_TURN_ANGLE; // if heading is bigger than this - deaccelerate and turn to reverse (instead of doing 180 turn)
    public static final double MAX_TURNING_RADIUS = 0.8; // if need to change direction and fast - reduce velocity to this radius
    public static final double MAX_ROTATION_VELOCITY = Math.sqrt(MAX_RADIAL_ACCEL * MAX_TURNING_RADIUS); // max velocity to use the preferred radius

}