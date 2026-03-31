// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotPose.Vision.VisionConstants.Camera;

/** Add your docs here. */
public class TagCamera {
    private Camera camera;
    private NetworkTable table;

    private double camToTagYaw;
    private double camToTagPitch;
    private double tagID;

    public TagCamera(Camera camera) {
        this.camera = camera;
        this.table = NetworkTableInstance.getDefault().getTable(camera.name());
    }

    private void updateValues() {
        camToTagPitch = table.getEntry("ty").getDouble(0.0);
        camToTagYaw = table.getEntry("tx").getDouble(0.0);
        tagID = table.getEntry("tid").getDouble(0.0);

        // need to take into account camera roll;

    }

    private double calculateCameraToTagDistance(double deltaHeight) {
        double alpha = camera.pitchInRadians() + camToTagPitch;
        return Math.abs(deltaHeight / Math.tan(alpha));
    }

    private Translation2d getCameraToTag(double deltaHeight) {
        return new Translation2d(calculateCameraToTagDistance(deltaHeight), camToTagYaw);
    }

    private Translation2d getRobotToTag(Rotation2d gyroAngle, double deltaHeight) {
        Translation2d cameraToTag = getCameraToTag(deltaHeight);
        return camera.robotToCamPosition().toTranslation2d().plus(cameraToTag.rotateBy(gyroAngle));
    }

    private Translation2d getOriginToRobot(Rotation2d gyroAngle) {
        Translation3d tagPosition = getTagPosition(tagID); // need to add function
        double deltaHeight = tagPosition.getZ() - camera.robotToCamPosition().getZ();

        return tagPosition.toTranslation2d().minus(getRobotToTag(gyroAngle, deltaHeight));

    }

    private boolean isSeeTag() {
        return table.getEntry("tv").getDouble(0.0) > 0.1;
    }

    public Pose2d getPose(Rotation2d gyroAngle) {
        if (!isSeeTag())
            return Pose2d.kZero;

        return new Pose2d(getOriginToRobot(gyroAngle), gyroAngle);

    }

}
