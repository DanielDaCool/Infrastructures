// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose;

import static frc.robot.RobotPose.RobotPoseConstants.COLLISION_ODOMETRY_STD;
import static frc.robot.RobotPose.RobotPoseConstants.DEFAULT_ODOMETRY_STD;
import static frc.robot.RobotPose.RobotPoseConstants.DEFAULT_QUEST_STD;
import static frc.robot.RobotPose.RobotPoseConstants.DEFAULT_VISION_STD;
import static frc.robot.RobotPose.RobotPoseConstants.G_FOR_COLLISION;
import static frc.robot.RobotPose.RobotPoseConstants.TIME_AFTER_COLLISION_FOR_RESET_STD;

import java.util.function.Supplier;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotPose.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Vision.Quest;
import frc.robot.RobotPose.Vision.TagCamera;
import frc.robot.RobotPose.Vision.VisionConstants;

/** Add your docs here. */
public class RobotPose {

    private final TagCamera[] aprilTagCameras;
    private final DemaciaPoseEstimator poseEstimator;
    private final Quest quest;
    // private final Supplier<Rotation2d> gyroAngleSupplier;
    private final Supplier<OdometryData> odometryDataSupplier;
    private boolean hasUpdatedQuestPose;

    private Matrix<N3, N1> visionSTD;
    private Matrix<N3, N1> questSTD;
    private Matrix<N3, N1> odometrySTD;

    private final BuiltInAccelerometer accelerometer;
    private final boolean useQuest;
    private final Timer afterCollisionTimer;

    private boolean hasStartedCollisionTimer;
    private static RobotPose instance;

    private RobotPose(SwerveModulePosition[] initialModulePositions, Supplier<OdometryData> odometryDataSupplier,
            Matrix<N3, N1> visionSTD, Matrix<N3, N1> odometrySTD, Matrix<N3, N1> questSTD,
            boolean useQuest, TagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;

        this.poseEstimator = new DemaciaPoseEstimator(initialModulePositions, odometrySTD, visionSTD);
        this.quest = new Quest(VisionConstants.ROBOT_TO_QUEST);
        this.odometryDataSupplier = odometryDataSupplier;
        this.useQuest = useQuest;
        if (useQuest)
            this.questSTD = questSTD;
        this.odometrySTD = odometrySTD;
        this.visionSTD = visionSTD;
        this.accelerometer = new BuiltInAccelerometer();
        this.afterCollisionTimer = new Timer();
        afterCollisionTimer.reset();
        this.hasStartedCollisionTimer = false;
        this.hasUpdatedQuestPose = false;
    }

    public void resetQuestPose(Pose2d questPose) {
        hasUpdatedQuestPose = true;
        quest.setQuestPose(new Pose3d(questPose));

    }

    public static void initialize(Supplier<OdometryData> odometryDataSupplier,
            TagCamera... aprilTagCameras) {

        initialize(odometryDataSupplier, true, aprilTagCameras);
    }

    public static void initialize(Supplier<OdometryData> odometryDataSupplier, boolean useQuest,
            TagCamera... aprilTagCameras) {
        SwerveModulePosition[] kZeroPositions = new SwerveModulePosition[4];
        for (int i = 0; i < kZeroPositions.length; i++) {
            kZeroPositions[i] = new SwerveModulePosition();
        }
        initialize(kZeroPositions, odometryDataSupplier, useQuest, aprilTagCameras);
    }

    public static void initialize(SwerveModulePosition[] initialModulePositions,
            Supplier<OdometryData> odometryDataSupplier, boolean useQuest, TagCamera... aprilTagCameras) {
        initialize(initialModulePositions, odometryDataSupplier, DEFAULT_VISION_STD, DEFAULT_ODOMETRY_STD,
                DEFAULT_QUEST_STD, useQuest, aprilTagCameras);
    }

    public static void initialize(SwerveModulePosition[] initialModulePositions,
            Supplier<OdometryData> odometryDataSupplier, Matrix<N3, N1> visionSTD, Matrix<N3, N1> odometrySTD,
            Matrix<N3, N1> questSTD, boolean useQuest, TagCamera... aprilTagCameras) {

        instance = new RobotPose(initialModulePositions, odometryDataSupplier, visionSTD, odometrySTD, questSTD,
                useQuest,
                aprilTagCameras);
    }

    private boolean shouldUpdateQuest() {
        return useQuest && quest.isConnected() && quest.isTracking() && hasUpdatedQuestPose;
    }

    private boolean isColliding() {
        return Math.abs(accelerometer.getX()) >= G_FOR_COLLISION || Math.abs(accelerometer.getY()) >= G_FOR_COLLISION;
    }

    public void peridioc() {
        if (useQuest && quest.isConnected() && quest.isTracking())
            quest.periodic();

        if (isColliding() && !hasStartedCollisionTimer) {
            hasStartedCollisionTimer = true;
            afterCollisionTimer.start();
            poseEstimator.setStateStd(COLLISION_ODOMETRY_STD);
        }

        if (afterCollisionTimer.hasElapsed(TIME_AFTER_COLLISION_FOR_RESET_STD)) {
            hasStartedCollisionTimer = false;
            poseEstimator.setStateStd(odometrySTD);
            afterCollisionTimer.stop();
            afterCollisionTimer.reset();

        }
        poseEstimator.addOdometryObservation(odometryDataSupplier.get());

        poseEstimator.setVisionMeasurementStdDevs(visionSTD);
        for (TagCamera camera : aprilTagCameras) {
            camera.periodic();
            if (camera.isSeeTag()) {
                poseEstimator.addVisionMeasurement(camera.getPose(odometryDataSupplier.get().gyroAngle()),
                        Timer.getFPGATimestamp() - 0.05);
            }
        }

        if (shouldUpdateQuest()) {
            poseEstimator.setVisionMeasurementStdDevs(questSTD);
            poseEstimator.addVisionMeasurement(quest.getRobotPose(), quest.getTimestamp());
        }
    }

    public static synchronized RobotPose getInstance() {
        return instance;
    }

}
