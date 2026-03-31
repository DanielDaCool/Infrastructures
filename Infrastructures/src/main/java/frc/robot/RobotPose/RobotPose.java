// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class RobotPose {

    private DemaciaOdometry odometry;
    private static RobotPose instance;
    private RobotPose(){
        


    }

    public static void initialize(SwerveModulePosition[] initialModulePositions){
        DemaciaOdometry.initialize(initialModulePositions);
        instance = new RobotPose();
    }

    public static synchronized RobotPose getInstance(){
        return instance;
    }


}
