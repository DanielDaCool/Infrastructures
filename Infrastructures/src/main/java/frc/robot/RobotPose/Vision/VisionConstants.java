// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {
    public record Camera(String name,Translation3d robotToCamPosition, double pitchInRadians, double yawInRadians, double rollInRadinas) {
    }

}
