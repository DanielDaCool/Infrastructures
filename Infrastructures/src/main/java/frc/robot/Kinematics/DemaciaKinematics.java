// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Kinematics.KinematicsConstants.*;

/**
 * Swerve kinematics for the DemaciaCode chassis: replaces {@code frc.demacia.kinematics.DemaciaKinematics}
 * with only the import changed (same constructor, {@link #toSwerveModuleStates(ChassisSpeeds)} and
 * {@link #toChassisSpeeds(SwerveModuleState[], double)}).
 *
 * <p>{@link #toSwerveModuleStates(ChassisSpeeds)} takes omega clockwise-positive, like the chassis
 * sends it (the old class computed {@code vy - omega * x}). Everything else is in the WPILib frame:
 * module positions x forward, y left; {@link #toChassisSpeeds} returns omega counter-clockwise
 * positive (the gyro's).
 *
 * <p>What it fixes over plain kinematics (the module states are computed for the loop the robot is
 * really in, not as if the modules were already there):
 * <ul>
 * <li>the robot turns during the loop and the command arrives late: the speeds are discretized over
 * the loop plus the latency ({@link KinematicsConstants#LATENCY}), so a robot that drives and spins
 * stays on its path;</li>
 * <li>with the measured module states ({@link #toSwerveModuleStates(ChassisSpeeds, SwerveModuleState[])},
 * one added argument in the chassis) a module that still points the wrong way gets less speed
 * (cosine scaling) instead of pushing the robot off the path;</li>
 * <li>a standing robot keeps its module angles (no snap to 0).</li>
 * </ul>
 * Optional: {@link #getDriveAccelerations()} and {@link #getSteerVelocities()} for the drive and steer
 * feed-forwards.
 *
 * <p>Stateful: call one of the {@code toSwerveModuleStates} methods once per loop.
 * {@link #toWpilibModuleStates} is the plain (stateless) WPILib inverse kinematics.
 */
public class DemaciaKinematics {

    private static final double MIN_MODULE_VELOCITY = 1e-9; // slower has no direction, angle is 0 [m/s]

    private Translation2d[] modulePositionOnTheRobot;
    private Rotation2d[] lastAngles; // angles sent last loop
    private double[] lastVx, lastVy; // module velocity vectors sent last loop [m/s]
    private double[] driveAccelerations; // [m/s^2]
    private double[] steerVelocities; // [rad/s]
    private Translation2d center; // average of the module positions
    private double sumSquaredDistance; // sum of the squared module distances from the center

    public DemaciaKinematics(Translation2d... modulePositionOnTheRobot) {
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
        int n = modulePositionOnTheRobot.length;
        lastAngles = new Rotation2d[n];
        Arrays.fill(lastAngles, Rotation2d.kZero);
        lastVx = new double[n];
        lastVy = new double[n];
        driveAccelerations = new double[n];
        steerVelocities = new double[n];

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
     * Module states for the chassis, once per loop.
     *
     * @param wantedSpeeds robot-relative speeds, omega clockwise-positive (as the DemaciaCode chassis
     *                     sends it)
     * @return module states, in the same order as the module positions
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds wantedSpeeds) {
        return send(plan(wantedSpeeds));
    }

    /**
     * {@link #toSwerveModuleStates(ChassisSpeeds)} that also takes the measured module states: each
     * module is turned to the side closer to its measured angle (reversing the wheel instead of turning
     * more than 90 degrees) and its speed is scaled by the cosine of the angle it still has to turn,
     * so a module that points the wrong way doesn't push the robot off the path (0 at 90 degrees or
     * more). In the chassis: {@code toSwerveModuleStates(speeds, getModuleStates())}.
     *
     * @param wantedSpeeds  robot-relative speeds, omega clockwise-positive
     * @param currentStates measured module states (only the angle is used)
     * @return module states, in the same order as the module positions
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds wantedSpeeds, SwerveModuleState[] currentStates) {
        SwerveModuleState[] states = plan(wantedSpeeds);
        for (int i = 0; i < states.length; i++) {
            Rotation2d measured = currentStates[i].angle;
            double off = states[i].angle.minus(measured).getRadians();
            if (Math.abs(off) > Math.PI / 2) {
                states[i].angle = new Rotation2d(-states[i].angle.getCos(), -states[i].angle.getSin());
                states[i].speedMetersPerSecond = -states[i].speedMetersPerSecond;
            }
            states[i].speedMetersPerSecond *= states[i].angle.minus(measured).getCos();
        }
        return send(states);
    }

    /**
     * The drive acceleration of each module for the states returned last, from the change of the
     * commanded module velocity over one loop (never from the measured speed), clamped to
     * {@link KinematicsConstants#MAX_DRIVE_ACCEL}. Same sign as the state's speed. Send it as the drive
     * motor's acceleration feed-forward (Phoenix 6 {@code VelocityVoltage.withAcceleration}, or
     * {@code kA * a}). Only with {@link #toSwerveModuleStates(ChassisSpeeds, SwerveModuleState[])}: the
     * module must not reverse the state again, or the sign is wrong.
     *
     * @return accelerations [m/s^2], in the same order as the module positions
     */
    public double[] getDriveAccelerations() {
        return driveAccelerations.clone();
    }

    /**
     * The steer velocity of each module for the states returned last: the change of the commanded
     * angle over one loop, clamped to {@link KinematicsConstants#MAX_STEER_RATE}. Send it as the steer
     * motor's velocity feed-forward when the steer runs {@code PositionVoltage} (Phoenix 6
     * {@code PositionVoltage.withVelocity}, in rotations/s, or {@code kV * velocity} volts); with
     * MotionMagic don't use it. Only with {@link #toSwerveModuleStates(ChassisSpeeds, SwerveModuleState[])}.
     *
     * @return steer velocities [rad/s], counter-clockwise positive, in the same order as the module
     *         positions
     */
    public double[] getSteerVelocities() {
        return steerVelocities.clone();
    }

    /** Desaturate, discretize over the loop and the latency, inverse kinematics; stand still when slow. */
    private SwerveModuleState[] plan(ChassisSpeeds wantedSpeeds) {
        // the chassis sends omega clockwise-positive; the math below is WPILib (counter-clockwise)
        ChassisSpeeds speeds = new ChassisSpeeds(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond,
                -wantedSpeeds.omegaRadiansPerSecond);
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < MIN_VELOCITY
                && Math.abs(speeds.omegaRadiansPerSecond) < MIN_VELOCITY) {
            SwerveModuleState[] standing = new SwerveModuleState[lastAngles.length];
            for (int i = 0; i < standing.length; i++) {
                standing[i] = new SwerveModuleState(0, lastAngles[i]);
            }
            return standing;
        }
        ChassisSpeeds desaturated = toChassisSpeeds(toWpilibModuleStates(speeds));
        return toWpilibModuleStates(ChassisSpeeds.discretize(desaturated, CYCLE_DT + 2 * LATENCY));
    }

    /** Keeps what is sent for the next loop and computes the feed-forwards. */
    private SwerveModuleState[] send(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            double cos = states[i].angle.getCos();
            double sin = states[i].angle.getSin();
            double speed = states[i].speedMetersPerSecond;
            // the change of the module velocity along the new direction
            double acceleration = (speed - (lastVx[i] * cos + lastVy[i] * sin)) / CYCLE_DT;
            driveAccelerations[i] = MathUtil.clamp(acceleration, -MAX_DRIVE_ACCEL, MAX_DRIVE_ACCEL);
            lastVx[i] = speed * cos;
            lastVy[i] = speed * sin;
            steerVelocities[i] = MathUtil.clamp(states[i].angle.minus(lastAngles[i]).getRadians() / CYCLE_DT,
                    -MAX_STEER_RATE, MAX_STEER_RATE);
            lastAngles[i] = states[i].angle;
        }
        return states;
    }

    /**
     * Plain WPILib inverse kinematics (omega counter-clockwise-positive, no discretize, stateless):
     * module states for robot-relative speeds, desaturated so no module is faster than
     * MAX_ALLOWED_MODULE_VELOCITY. All modules are scaled by the same factor, so the robot keeps
     * the direction of motion and the ratio between translation and rotation.
     *
     * @param wantedSpeeds robot-relative speeds
     * @return new module states, in the same order as the module positions
     */
    public SwerveModuleState[] toWpilibModuleStates(ChassisSpeeds wantedSpeeds) {
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
