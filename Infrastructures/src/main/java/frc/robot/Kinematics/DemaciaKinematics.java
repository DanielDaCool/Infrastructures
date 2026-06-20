// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Kinematics.KinematicsConstants.*;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    private SwerveModuleState[] lastStates = new SwerveModuleState[4];
    private ChassisSpeeds lastWantedSpeeds;

    public DemaciaKinematics(Translation2d... modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        lastWantedSpeeds = new ChassisSpeeds();
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
        for (int i = 0; i < 4; i++) {
            swerveStates[i] = new SwerveModuleState();
            lastStates[i] = new SwerveModuleState();
        }

    }

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] swerveStates, double omegaFromGyro) {
        double sumVx = 0;
        double sumVy = 0;
        for (int i = 0; i < 4; i++) {
            double angleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double distanceFromCenter = modulePositionOnTheRobot[i].getNorm();
            double currentAngle = swerveStates[i].angle.getRadians();
            double moduleVx = swerveStates[i].speedMetersPerSecond * Math.cos(currentAngle);
            double moduleVy = swerveStates[i].speedMetersPerSecond * Math.sin(currentAngle);

            double chassisVx = moduleVx - (omegaFromGyro * distanceFromCenter
                    * Math.sin(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));
            double chassisVy = moduleVy + (omegaFromGyro * distanceFromCenter
                    * Math.cos(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));

            sumVx += chassisVx;
            sumVy += chassisVy;
        }
        return new ChassisSpeeds(sumVx / 4.0, sumVy / 4.0, omegaFromGyro);
    }


    public SwerveModuleState[] toSwerveModuleStatesWithAccel(ChassisSpeeds wantedSpeeds){
        return toSwerveModuleStatesWithAccel(wantedSpeeds, KinematicsUtilities.getAccelFromDelta(wantedSpeeds, lastWantedSpeeds));

    }
    public SwerveModuleState[] toSwerveModuleStatesWithAccel(ChassisSpeeds wantedSpeeds, ChassisAccel wantedAccel) {
        double omega = wantedSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < 4; i++) {
            double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();

            double wantedModuleVx = wantedSpeeds.vxMetersPerSecond + (wantedAccel.accelX() * CYCLE_DT)
                    - (modulePositionOnTheRobot[i].getNorm() * (omega + wantedAccel.accelOmega() * CYCLE_DT)
                            * Math.sin((omega * CYCLE_DT)
                                    + (0.5 * wantedAccel.accelOmega() * CYCLE_DT * CYCLE_DT
                                            + moduleAngleFromCenter)));

            double wantedModuleVy = wantedSpeeds.vyMetersPerSecond + (wantedAccel.accelY() * CYCLE_DT)
                    + (modulePositionOnTheRobot[i].getNorm() * (omega + wantedAccel.accelOmega() * CYCLE_DT)
                            * Math.cos((omega * CYCLE_DT)
                                    + (0.5 * wantedAccel.accelOmega() * CYCLE_DT * CYCLE_DT
                                            + moduleAngleFromCenter)));

            swerveStates[i] = new SwerveModuleState(Math.hypot(wantedModuleVx, wantedModuleVy),
                    new Rotation2d(wantedModuleVx, wantedModuleVy));
        }

        lastWantedSpeeds = wantedSpeeds;
        swerveStates = factorModuleVelocities(swerveStates);
        return swerveStates;

    }

    // public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds wantedSpeeds) {

    //     double omega = wantedSpeeds.omegaRadiansPerSecond;

    //     for (int i = 0; i < 4; i++) {
    //         double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
    //         double moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
    //         Translation2d velocityVector = new Translation2d(
    //                 wantedSpeeds.vxMetersPerSecond + omega * modulePositionOnTheRobot[i].getNorm()
    //                         * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter),
    //                 wantedSpeeds.vyMetersPerSecond - omega * modulePositionOnTheRobot[i].getNorm()
    //                         * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter));
    //         swerveStates[i] = new SwerveModuleState(velocityVector.getNorm(), new Rotation2d(
    //                 KinematicsUtilities.getAngleFromVector(velocityVector.getX(), velocityVector.getY())));
    //     }

    //     swerveStates = factorModuleVelocities(swerveStates);
    //     return swerveStates;
    // }

    private SwerveModuleState[] factorModuleVelocities(SwerveModuleState[] swerveStates) {
        double maxVelocityCalculated = 0;
        for (int i = 0; i < swerveStates.length; i++) {
            double cur = Math.abs(swerveStates[i].speedMetersPerSecond);
            if (cur == 0)
                return swerveStates;
            if (cur > maxVelocityCalculated)
                maxVelocityCalculated = cur;
        }
        double factor = MAX_ALLOWED_MODULE_VELOCITY / maxVelocityCalculated;

        if (factor >= 1)
            return swerveStates;

        for (SwerveModuleState state : swerveStates) {
            state.speedMetersPerSecond = state.speedMetersPerSecond * factor;
        }
        return swerveStates;

    }

}