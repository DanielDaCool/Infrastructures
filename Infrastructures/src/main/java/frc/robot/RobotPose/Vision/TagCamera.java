// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
    private double latency;

    public TagCamera(Camera camera) {
        this.camera = camera;
        this.table = NetworkTableInstance.getDefault().getTable(camera.name());
        updateValues();
    }

    public void periodic() {
        updateValues();
    }

    private void updateValues() {
        camToTagPitch = Math.toRadians(table.getEntry("ty").getDouble(0.0));
        camToTagYaw = Math.toRadians(table.getEntry("tx").getDouble(0.0));
        tagID = table.getEntry("tid").getDouble(0.0);

        // need to take into account camera roll;

        latency = table.getEntry("tl").getDouble(0.0);

    }

    private double calculateCameraToTagDistance(double deltaHeight) {
        double alpha = camera.pitchInRadians() + camToTagPitch;
        double distance = Math.abs(deltaHeight / Math.tan(alpha)) / Math.cos((camToTagYaw + camera.yawInRadians()));

        System.out.println("Distance: " + distance);

        return distance;    
    }

    private Translation2d getCameraToTag(double deltaHeight) {
        return new Translation2d(calculateCameraToTagDistance(deltaHeight),
                new Rotation2d(camToTagYaw + camera.yawInRadians()));
    }

    private Translation2d getRobotToTag(Rotation2d gyroAngle, double deltaHeight) {
        Translation2d cameraToTag = getCameraToTag(deltaHeight);
        return camera.robotToCamPosition().toTranslation2d().plus(cameraToTag.rotateBy(gyroAngle));
    }

    private Translation2d getOriginToRobot(Rotation2d gyroAngle) {
        Translation3d tagPosition = getTagPosition((int) tagID);
        double deltaHeight = tagPosition.getZ() - camera.robotToCamPosition().getZ();

        return tagPosition.toTranslation2d().minus(getRobotToTag(gyroAngle, deltaHeight));

    }

    public double getLatency() {
        return latency;
    }

    public boolean isSeeTag() {
        return table.getEntry("tv").getDouble(0.0) > 0.1;
    }

    private Translation3d getTagPosition(int tagID) {
        return AprilTagFieldLayout.loadField(VisionConstants.APRIL_TAG_FIELD).getTagPose(tagID).get()
                .getTranslation();
    }

    public Pose2d getPose(Rotation2d gyroAngle) {
        if (!isSeeTag())
            return Pose2d.kZero;

        return new Pose2d(getOriginToRobot(gyroAngle), gyroAngle);

    }

}
