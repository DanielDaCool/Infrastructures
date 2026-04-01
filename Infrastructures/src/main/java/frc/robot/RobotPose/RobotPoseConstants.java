// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class RobotPoseConstants {

    public static final double G_FOR_COLLISION = 1.8;

    public static final Matrix<N3, N1> DEFAULT_VISION_STD = VecBuilder.fill(0.1, 0.1, Integer.MAX_VALUE);
    public static final Matrix<N3, N1> DEFAULT_QUEST_STD = VecBuilder.fill(0.01, 0.01, Integer.MAX_VALUE);
    public static final Matrix<N3, N1> DEFAULT_ODOMETRY_STD = VecBuilder.fill(0.05, 0.05, 0);

    public static final Matrix<N3, N1> COLLISION_ODOMETRY_STD = VecBuilder.fill(0.15, 0.15, 0);
    public static final double TIME_AFTER_COLLISION_FOR_RESET_STD = 0.3;
}
