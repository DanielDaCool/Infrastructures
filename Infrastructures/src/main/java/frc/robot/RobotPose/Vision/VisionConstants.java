// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {
    public record Camera(String name, Translation3d robotToCamPosition, double pitchInRadians, double yawInRadians,
            double rollInRadians) {
    }

    public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2026RebuiltAndymark;

    public static final Transform3d ROBOT_TO_QUEST = new Transform3d();

    public static final TagCamera[] APRIL_TAG_CAMERAS = {
            new TagCamera(new Camera("limelight-hub", new Translation3d(0.27,-0.20,0.355), Math.toRadians(18.5), Math.toRadians(2), 0))
    };
}
