// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

import edu.wpi.first.math.MathUtil;
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

    private static final double REVERSE_BAND = Math.toRadians(10); // hysteresis around the 90 degree reverse limit
    private static final double MIN_REVERSE_SPEED = 1e-3; // a slower module has no direction to decide with [m/s]

    /**
     * Module commands for robot speeds that are already discretized: the module states
     * (desaturated, as {@link #toSwerveModuleStates}), then per module
     * <ol>
     * <li>the reverse decision against the previous commanded angle, with a 10 degree
     * hysteresis (below);</li>
     * <li>cosine scaling of the speed against the measured angle advanced by its latency;</li>
     * <li>the drive acceleration feed-forward (change of the planned speed over dt, clamped) and,
     * when enabled, the steer velocity feed-forward (change of the commanded angle over dt,
     * clamped).</li>
     * </ol>
     * Stateless: the state is in {@code previous} and {@code reversed}, which the caller keeps
     * (see {@link AccelLimitedSwerve}).
     *
     * <p>Reverse decision: a module reverses (angle + 180 degrees, speed negated) when its target
     * is more than 90 degrees from its previous commanded angle. A module within 10 degrees of
     * that limit does not decide alone: the modules in the band take the majority decision, and
     * on a tie each keeps its own previous reversed flag, so a robot driving with the modules
     * near 90 degrees does not flip some wheels and not others. A module with no speed keeps its
     * previous angle.
     *
     * <p>A NaN or infinite measured angle or steer velocity makes that module use its previous
     * commanded angle as the measured angle and a steer velocity of 0, so no cosine scaling is
     * done on garbage.
     *
     * @param discretizedRobotRel     robot-relative speeds, discretized by the caller
     * @param measuredAngles          measured module angles
     * @param measuredSteerVelocities measured steer velocities [rad/s]
     * @param previous                the commands sent last loop; the speed and angle are the
     *                                ones after reversing and scaling
     * @param reversed                IN/OUT: whether each module was reversed last loop. Read for
     *                                the tie break, and set to this loop's decision
     * @param config                  latencies and limits
     * @param dt                      loop time [s]
     * @return the commands, in the order of the module positions
     * @throws IllegalArgumentException when an array does not have one entry per module or dt is
     *                                  not positive and finite
     */
    public ModuleCommand[] toModuleCommands(ChassisSpeeds discretizedRobotRel, Rotation2d[] measuredAngles,
            double[] measuredSteerVelocities, ModuleCommand[] previous, boolean[] reversed,
            ModuleCommandConfig config, double dt) {
        int n = modulePositionOnTheRobot.length;
        if (measuredAngles.length != n || measuredSteerVelocities.length != n || previous.length != n
                || reversed.length != n) {
            throw new IllegalArgumentException("every array needs one entry per module (" + n + ")");
        }
        if (!(dt > 0 && Double.isFinite(dt))) {
            throw new IllegalArgumentException("dt must be finite and > 0: " + dt);
        }
        SwerveModuleState[] targets = toSwerveModuleStates(discretizedRobotRel);

        // reverse decision against the previous commanded angle
        boolean[] flip = new boolean[n];
        boolean[] inBand = new boolean[n];
        int bandCount = 0, bandFlips = 0;
        for (int i = 0; i < n; i++) {
            double off = Math.abs(targets[i].angle.minus(previous[i].angle()).getRadians());
            flip[i] = off > Math.PI / 2;
            inBand[i] = Math.abs(off - Math.PI / 2) <= REVERSE_BAND
                    && Math.abs(targets[i].speedMetersPerSecond) > MIN_REVERSE_SPEED;
            if (inBand[i]) {
                bandCount++;
                if (flip[i]) {
                    bandFlips++;
                }
            }
        }
        for (int i = 0; i < n; i++) {
            if (inBand[i]) {
                flip[i] = 2 * bandFlips != bandCount ? 2 * bandFlips > bandCount : reversed[i];
            }
        }

        ModuleCommand[] commands = new ModuleCommand[n];
        for (int i = 0; i < n; i++) {
            Rotation2d angle = targets[i].angle;
            double speed = targets[i].speedMetersPerSecond;
            if (Math.abs(speed) <= MIN_MODULE_VELOCITY) {
                angle = previous[i].angle(); // no direction: keep the angle
                flip[i] = reversed[i];
                speed = 0;
            } else if (flip[i]) {
                angle = new Rotation2d(-angle.getCos(), -angle.getSin());
                speed = -speed;
            }
            reversed[i] = flip[i];

            // cosine scaling against the measured angle advanced by its latency
            Rotation2d measured = measuredAngles[i];
            double steerVelocity = measuredSteerVelocities[i];
            Rotation2d advanced;
            if (measured != null && Double.isFinite(measured.getCos()) && Double.isFinite(measured.getSin())
                    && Double.isFinite(steerVelocity)) {
                advanced = measured.plus(new Rotation2d(steerVelocity * config.measuredAngleLatencySeconds()));
            } else {
                advanced = previous[i].angle();
            }
            speed *= Math.max(0, angle.minus(advanced).getCos());

            double maxAccel = config.maxDriveAccelMetersPerSecondSquared();
            double acceleration = MathUtil.clamp((speed - previous[i].speedMetersPerSecond()) / dt, -maxAccel,
                    maxAccel);
            double steerFeedforward = 0;
            if (config.steerVelocityFeedforward()) {
                double maxRate = config.maxSteerRateRadiansPerSecond();
                steerFeedforward = MathUtil.clamp(angle.minus(previous[i].angle()).getRadians() / dt, -maxRate,
                        maxRate);
            }
            commands[i] = new ModuleCommand(angle, speed, acceleration, steerFeedforward);
        }
        return commands;
    }

    /**
     * Commands for a robot that should stand: every module keeps its previous angle, speed 0, and
     * the drive acceleration feed-forward brakes from the previous speed (clamped).
     *
     * @param previous the commands sent last loop
     * @param config   limits
     * @param dt       loop time [s]
     */
    public static ModuleCommand[] toStandingCommands(ModuleCommand[] previous, ModuleCommandConfig config,
            double dt) {
        ModuleCommand[] commands = new ModuleCommand[previous.length];
        double maxAccel = config.maxDriveAccelMetersPerSecondSquared();
        for (int i = 0; i < commands.length; i++) {
            commands[i] = new ModuleCommand(previous[i].angle(), 0,
                    MathUtil.clamp(-previous[i].speedMetersPerSecond() / dt, -maxAccel, maxAccel), 0);
        }
        return commands;
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
