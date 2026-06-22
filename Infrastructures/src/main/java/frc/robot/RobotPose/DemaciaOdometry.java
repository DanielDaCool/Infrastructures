// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import static frc.robot.RobotPose.RobotPoseConstants.*;

/** Add your docs here. */
public class DemaciaOdometry {

    private Pose2d pose;
    private SwerveModulePosition[] lastModulePositions;
    private static DemaciaOdometry instance;
    private Translation2d[] moduleDisplacements;

    private double[] modulesWeights;

    private DemaciaOdometry(SwerveModulePosition[] initialModulePositions) {
        this.lastModulePositions = new SwerveModulePosition[initialModulePositions.length];
        this.modulesWeights = new double[initialModulePositions.length];

        this.moduleDisplacements = new Translation2d[initialModulePositions.length];

        lastModulePositions = initialModulePositions;
        for (int i = 0; i < lastModulePositions.length; i++) {
            modulesWeights[i] = 1.0 / initialModulePositions.length;
            moduleDisplacements[i] = Translation2d.kZero;
        }
        this.pose = Pose2d.kZero;

    }
    public static synchronized DemaciaOdometry getInstance(SwerveModulePosition[] initialModulePositions) {
        if(instance == null) instance = new DemaciaOdometry(initialModulePositions);
        return instance;
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        for (int i = 0; i < modulePositions.length; i++) {
            moduleDisplacements[i] = calculateModuleDisplacement(lastModulePositions[i], modulePositions[i]);
        }

        Translation2d robotDisplacement = calculateRobotDisplacement(moduleDisplacements);
        pose = new Pose2d(pose.getTranslation().plus(robotDisplacement), gyroAngle);

        lastModulePositions = modulePositions;
    }

    private Translation2d calculateModuleDisplacement(SwerveModulePosition lastPosition,
            SwerveModulePosition currentPosition) {
        double arcLength = currentPosition.distanceMeters - lastPosition.distanceMeters;
        double deltaAngle = currentPosition.angle.getRadians() - lastPosition.angle.getRadians();
        if (Math.abs(Math.toDegrees(deltaAngle)) < 1E-6)
            return new Translation2d(arcLength, currentPosition.angle); // case for almost straight line

        double centralAngle = deltaAngle;
        double radius = arcLength / centralAngle;
        double chordLength = 2 * radius * Math.sin(centralAngle * 0.5);
        Rotation2d chordAngle = lastPosition.angle.plus(new Rotation2d(centralAngle * 0.5));

        return new Translation2d(chordLength, chordAngle);
    }

    private Translation2d calculateRobotDisplacement(Translation2d[] moduleDisplacements) {
        double x = 0;
        double y = 0;

        for (int i = 0; i < moduleDisplacements.length; i++) {
            x += moduleDisplacements[i].getX() * modulesWeights[i];
            y += moduleDisplacements[i].getY() * modulesWeights[i];
        }

        return new Translation2d(x, y);

    }

    public void resetPose() {
        resetPose(Pose2d.kZero);
    }

    public void resetPose(Pose2d newPose) {
        this.pose = newPose;
    }

    public Pose2d getOdometryPose() {
        return this.pose;
    }

    public void changeModuleWeight(int index, double newWeight) {
        modulesWeights[index] = MathUtil.clamp(newWeight, 0, 1);
    }
}
