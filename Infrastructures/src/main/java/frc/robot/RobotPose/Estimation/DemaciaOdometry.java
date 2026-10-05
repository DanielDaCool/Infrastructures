// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Estimation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Swerve odometry: turns gyro + module encoder readings into how far the robot moved since
 * the last update.
 *
 * <p>Differences from WPILib's {@code SwerveDriveOdometry}:
 * <ul>
 * <li>Each module's path over one loop is treated as a circular arc (its wheel angle changed
 * during the loop), and the straight-line chord of that arc is used as its displacement.</li>
 * <li>The robot displacement is a weighted average of the module displacements, and the
 * weights can be changed at runtime with {@link #changeModuleWeight} (e.g. to trust a
 * slipping module less).</li>
 * <li>The heading always comes from the gyro, never from the wheels.</li>
 * </ul>
 *
 * <p>This class only does odometry. {@link DemaciaPoseEstimator} owns it and uses the twist
 * returned by {@link #updateOdometry} each loop.
 */
public class DemaciaOdometry {

    /** Pure-odometry pose (no vision). */
    private Pose2d pose;
    /** Field heading minus raw gyro reading, set on reset so the reset heading is kept. */
    private Rotation2d gyroOffset = Rotation2d.kZero;
    /** Module readings from the previous update, to get this loop's change. */
    private SwerveModulePosition[] lastModulePositions;
    /** Robot-relative chord of each module this loop (meters). Reused every update. */
    private Translation2d[] moduleDisplacements;
    /** Each module's weight in the displacement average; always sums to 1. Starts equal. */
    private double[] modulesWeights;
    /** Module locations relative to the robot centre (meters). */
    private final Translation2d[] moduleLocations;

    /**
     * @param initialModulePositions Module readings right now, so the first update only counts
     *                               motion from this point.
     * @param moduleLocations        Module positions relative to the robot center (meters),
     *                               same order as the module readings.
     */
    public DemaciaOdometry(SwerveModulePosition[] initialModulePositions, Translation2d[] moduleLocations) {
        this.moduleLocations = moduleLocations;
        this.lastModulePositions = new SwerveModulePosition[initialModulePositions.length];
        this.modulesWeights = new double[initialModulePositions.length];
        this.moduleDisplacements = new Translation2d[initialModulePositions.length];

        for (int i = 0; i < initialModulePositions.length; i++) {
            lastModulePositions[i] = initialModulePositions[i].copy();
            modulesWeights[i] = 1.0 / initialModulePositions.length;
            moduleDisplacements[i] = Translation2d.kZero;
        }
        this.pose = Pose2d.kZero;
    }

    /**
     * Updates the odometry with new gyro and module readings.
     *
     * <ol>
     * <li>Each module's displacement since the last update (robot-relative chord, see
     * {@link #calculateModuleDisplacement}).</li>
     * <li>dtheta from the gyro (with the reset offset), wrapped to [-pi, pi].</li>
     * <li>The robot-center displacement: weighted average of the modules, with the part
     * caused by rotation removed (see {@link #calculateRobotDisplacement}).</li>
     * <li>{@code pose = pose.exp(twist)}.</li>
     * </ol>
     *
     * The returned twist is exactly the one applied to {@code pose}, so the pose estimator can
     * replay it with {@code Pose2d.exp()} from any starting pose and get the same motion.
     *
     * @param gyroAngle       The current raw gyro heading.
     * @param modulePositions The current swerve module positions.
     * @return The robot-relative motion of this update (dx, dy in meters, dtheta in radians).
     */
    public Twist2d updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        for (int i = 0; i < modulePositions.length; i++) {
            moduleDisplacements[i] = calculateModuleDisplacement(lastModulePositions[i], modulePositions[i]);
        }

        Rotation2d previousRotation = pose.getRotation();
        double dtheta = MathUtil.angleModulus(gyroAngle.plus(gyroOffset).minus(previousRotation).getRadians());

        Translation2d robotDisplacementRobotRelative = calculateRobotDisplacement(moduleDisplacements, dtheta);

        Twist2d twist = new Twist2d(robotDisplacementRobotRelative.getX(),
                robotDisplacementRobotRelative.getY(), dtheta);

        pose = pose.exp(twist);

        for (int i = 0; i < lastModulePositions.length; i++) {
            lastModulePositions[i] = modulePositions[i].copy();
        }

        return twist;
    }

    /**
     * One module's straight-line displacement since the last update, robot-relative.
     *
     * <p>The wheel drove {@code arcLength} while its angle turned by {@code deltaAngle}, so it
     * went along an arc of radius {@code arcLength / deltaAngle}. The chord of that arc has
     * length {@code 2 * r * sin(deltaAngle / 2)} and points halfway between the old and new
     * wheel angles. When the angle barely changed it's a straight line at the current angle.
     */
    private Translation2d calculateModuleDisplacement(SwerveModulePosition lastPosition,
            SwerveModulePosition currentPosition) {
        double arcLength = currentPosition.distanceMeters - lastPosition.distanceMeters;
        double deltaAngle = MathUtil.angleModulus(currentPosition.angle.getRadians() - lastPosition.angle.getRadians());

        if (Math.abs(Math.toDegrees(deltaAngle)) < 1E-6)
            return new Translation2d(arcLength, currentPosition.angle); // case for almost straight line

        double centralAngle = deltaAngle;
        double radius = arcLength / centralAngle;
        double chordLength = 2 * radius * Math.sin(centralAngle * 0.5);
        Rotation2d chordAngle = lastPosition.angle.plus(new Rotation2d(centralAngle * 0.5));

        return new Translation2d(chordLength, chordAngle);
    }

    /**
     * Module i moves (dx - dtheta * y_i, dy + dtheta * x_i), so the weighted mean is
     * (dx, dy) + dtheta * (-y_w, x_w) where (x_w, y_w) is the weighted module centroid.
     * The rotation term is removed so non-uniform weights don't turn rotation into translation.
     */
    private Translation2d calculateRobotDisplacement(Translation2d[] moduleDisplacements, double dtheta) {
        double x = 0;
        double y = 0;
        for (int i = 0; i < moduleDisplacements.length; i++) {
            x += (moduleDisplacements[i].getX() + dtheta * moduleLocations[i].getY()) * modulesWeights[i];
            y += (moduleDisplacements[i].getY() - dtheta * moduleLocations[i].getX()) * modulesWeights[i];
        }
        return new Translation2d(x, y);
    }

    /**
     * Resets the pose. The gyro reading at the moment of the reset is needed so the new
     * heading is kept on the next update instead of being replaced by the raw gyro.
     *
     * @param newPose   The new field pose.
     */
    public void resetPose(Pose2d newPose, Rotation2d angle) {
        this.pose = newPose;
        this.gyroOffset = newPose.getRotation().minus(angle);
    }

    /** @return The pure-odometry pose (no vision corrections). */
    public Pose2d getOdometryPose() {
        return this.pose;
    }

    /**
     * Sets a module's weight in the robot-displacement average, then renormalizes every
     * other module's weight so the full array still sums to 1. 
     *
     * @param index     The index of the module whose weight is being changed.
     * @param newWeight The new weight for that module, clamped to [0, 1].
     */
    public void changeModuleWeight(int index, double newWeight) {
        double clampedNewWeight = MathUtil.clamp(newWeight, 0, 1);

        double remainingWeightSum = 0;
        for (int i = 0; i < modulesWeights.length; i++) {
            if (i != index) {
                remainingWeightSum += modulesWeights[i];
            }
        }

        double weightToDistribute = 1.0 - clampedNewWeight;
        modulesWeights[index] = clampedNewWeight;

        for (int i = 0; i < modulesWeights.length; i++) {
            if (i != index) {
                modulesWeights[i] = remainingWeightSum > 0
                        ? (modulesWeights[i] / remainingWeightSum) * weightToDistribute
                        : weightToDistribute / (modulesWeights.length - 1);
            }
        }
    }
}