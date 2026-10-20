// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Kinematics.KinematicsConstants.*;

/**
 * Swerve kinematics, the same math as WPILib's SwerveDriveKinematics.
 *
 * <p>A module at (x, y) on the robot moves at (vx - omega * y, vy + omega * x) (robot-relative).
 * {@link #toChassisSpeeds} is the exact inverse of {@link #toSwerveModuleStates} (before
 * desaturation).
 *
 * <p>There is no lead-angle term (the old {@code omega * 0.02} added to the module angle): it
 * turned only the rotation part of each module vector, so the modules no longer agreed on one
 * rigid-body motion and the forward kinematics couldn't undo it. To make up for the robot
 * turning during a loop, discretize the speeds before calling this (WPILib's
 * {@code ChassisSpeeds.discretize(speeds, CYCLE_DT)}). {@link AccelLimitedSwerve} does this:
 * it desaturates, then discretizes.
 */
public class DemaciaKinematics {

    private static final double MIN_MODULE_VELOCITY = 1e-9; // slower has no direction, angle is 0 [m/s]

    private Translation2d[] modulePositionOnTheRobot;
    private Translation2d center; // average of the module positions
    private double sumSquaredDistance; // sum of the squared module distances from the center

    public DemaciaKinematics(Translation2d... modulePositionOnTheRobot) {
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;

        double sumX = 0, sumY = 0;
        for (Translation2d position : modulePositionOnTheRobot) {
            sumX += position.getX();
            sumY += position.getY();
        }
        center = new Translation2d(sumX / modulePositionOnTheRobot.length, sumY / modulePositionOnTheRobot.length);
        sumSquaredDistance = 0;
        for (Translation2d position : modulePositionOnTheRobot) {
            double distance = position.minus(center).getNorm();
            sumSquaredDistance += distance * distance;
        }
    }

    public int getModuleCount() {
        return modulePositionOnTheRobot.length;
    }

    /**
     * Robot speeds from the module states: the rigid-body motion closest (least squares) to the
     * module velocities, with omega from the gyro. For a symmetric layout vx and vy are the average
     * of the module velocity vectors.
     *
     * @param swerveStates  module states, in the same order as the module positions
     * @param omegaFromGyro robot angular velocity [rad/s]
     * @return robot-relative speeds
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] swerveStates, double omegaFromGyro) {
        double sumVx = 0;
        double sumVy = 0;
        for (SwerveModuleState state : swerveStates) {
            sumVx += state.speedMetersPerSecond * state.angle.getCos();
            sumVy += state.speedMetersPerSecond * state.angle.getSin();
        }
        int n = swerveStates.length;
        // module i moves at (vx - omega * y_i, vy + omega * x_i), so the average module velocity
        // is (vx - omega * center y, vy + omega * center x)
        return new ChassisSpeeds(sumVx / n + omegaFromGyro * center.getY(),
                sumVy / n - omegaFromGyro * center.getX(), omegaFromGyro);
    }

    /**
     * Robot speeds from the module states only (no gyro): the least squares rigid-body motion,
     * omega included.
     *
     * @param swerveStates module states, in the same order as the module positions
     * @return robot-relative speeds
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] swerveStates) {
        double sumCross = 0;
        for (int i = 0; i < swerveStates.length; i++) {
            Translation2d fromCenter = modulePositionOnTheRobot[i].minus(center);
            double moduleVx = swerveStates[i].speedMetersPerSecond * swerveStates[i].angle.getCos();
            double moduleVy = swerveStates[i].speedMetersPerSecond * swerveStates[i].angle.getSin();
            sumCross += fromCenter.getX() * moduleVy - fromCenter.getY() * moduleVx;
        }
        double omega = sumSquaredDistance > 0 ? sumCross / sumSquaredDistance : 0;
        return toChassisSpeeds(swerveStates, omega);
    }

    /**
     * Module states for robot-relative speeds, desaturated so no module is faster than
     * MAX_ALLOWED_MODULE_VELOCITY. All modules are scaled by the same factor, so the robot keeps
     * the direction of motion and the ratio between translation and rotation.
     *
     * @param wantedSpeeds robot-relative speeds
     * @return new module states, in the same order as the module positions
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds wantedSpeeds) {
        double omega = wantedSpeeds.omegaRadiansPerSecond;
        SwerveModuleState[] swerveStates = new SwerveModuleState[modulePositionOnTheRobot.length];

        for (int i = 0; i < swerveStates.length; i++) {
            double moduleVx = wantedSpeeds.vxMetersPerSecond - omega * modulePositionOnTheRobot[i].getY();
            double moduleVy = wantedSpeeds.vyMetersPerSecond + omega * modulePositionOnTheRobot[i].getX();
            double moduleVelocity = Math.hypot(moduleVx, moduleVy);
            swerveStates[i] = new SwerveModuleState(moduleVelocity,
                    moduleVelocity > MIN_MODULE_VELOCITY ? new Rotation2d(moduleVx, moduleVy) : Rotation2d.kZero);
        }

        return factorModuleVelocities(swerveStates);
    }

    private SwerveModuleState[] factorModuleVelocities(SwerveModuleState[] swerveStates) {
        double maxVelocityCalculated = 0;
        for (SwerveModuleState state : swerveStates) {
            maxVelocityCalculated = Math.max(maxVelocityCalculated, Math.abs(state.speedMetersPerSecond));
        }
        // a module at 0 stays 0 when scaled, so only skip when no module is too fast
        if (maxVelocityCalculated <= MAX_ALLOWED_MODULE_VELOCITY)
            return swerveStates;

        double factor = MAX_ALLOWED_MODULE_VELOCITY / maxVelocityCalculated;
        for (SwerveModuleState state : swerveStates) {
            state.speedMetersPerSecond = state.speedMetersPerSecond * factor;
        }
        return swerveStates;

    }

}
