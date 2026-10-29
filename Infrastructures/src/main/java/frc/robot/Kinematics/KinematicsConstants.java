// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

/** Add your docs here. */
public class KinematicsConstants {

    public static final double MAX_ALLOWED_MODULE_VELOCITY = 4.5;
    public static final double CYCLE_DT = 0.02;
    
    public static final double MIN_VELOCITY = 0.01; // slower is 0

    // CAN/motor command latency + gyro latency, no half loop (the discretize over CYCLE_DT + 2 * LATENCY
    // already adds it). Placeholder: tune on the robot [s]
    public static final double LATENCY = 0.035;
    public static final double MAX_DRIVE_ACCEL = 20.0; // safety clamp of the drive acceleration feed-forward, above traction (the limiter sets the real limit) [m/s^2]
    public static final double MAX_STEER_RATE = 12.0; // clamp of the steer velocity feed-forward: free steer speed [rad/s]

}
