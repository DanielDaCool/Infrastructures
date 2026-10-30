// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotPose.Estimation;

import java.util.Arrays;

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
 * <li>The robot displacement is the average of the module displacements.</li>
 * <li>The heading always comes from the gyro, never from the wheels.</li>
 * <li>Bad readings are not used (see {@link #updateOdometry}): a sample with a NaN or
 * infinite value moves nothing, and a module whose distance jumps by more than
 * {@link #MAX_MODULE_STEP_METERS} in one sample (an encoder that rebooted to 0) is left out
 * of that sample.</li>
 * </ul>
 *
 * <p>This class only does odometry. {@link DemaciaPoseEstimator} owns it and uses the twist
 * returned by {@link #updateOdometry} each loop.
 */
public class DemaciaOdometry {

    /**
     * A module can't drive this far between two samples (5 m/s for 0.1 s), so a bigger step is
     * an encoder that rebooted or glitched.
     */
    public static final double MAX_MODULE_STEP_METERS = 0.5;

    /** Pure-odometry pose (no vision). */
    private Pose2d pose;
    /** Field heading minus raw gyro reading, set on reset so the reset heading is kept. */
    private Rotation2d gyroOffset = Rotation2d.kZero;
    /** Module readings from the previous update, to get this update's change; null before the first. */
    private SwerveModulePosition[] lastModulePositions;
    /** Robot-relative chord of each module this loop (meters). Reused every update. */
    private Translation2d[] moduleDisplacements;
    /** Each module's weight in the displacement average this update; sums to 1. */
    private final double[] modulesWeights;
    /** Module locations relative to the robot centre (meters). */
    private final Translation2d[] moduleLocations;

    /**
     * The first update only stores the module readings (no translation), so motion is
     * counted from then on.
     *
     * @param moduleLocations Module positions relative to the robot center (meters), same
     *                        order as the module readings.
     */
    public DemaciaOdometry(Translation2d[] moduleLocations) {
        this.moduleLocations = moduleLocations;
        this.modulesWeights = new double[moduleLocations.length];
        this.moduleDisplacements = new Translation2d[moduleLocations.length];

        for (int i = 0; i < moduleLocations.length; i++) {
            modulesWeights[i] = 1.0 / moduleLocations.length;
            moduleDisplacements[i] = Translation2d.kZero;
        }
        this.pose = Pose2d.kZero;
    }

    /**
     * Updates the odometry with new gyro and module readings.
     *
     * <ol>
     * <li>If any gyro or module value is NaN or infinite, nothing moves and the readings are
     * not stored, so the next good sample covers the motion. (Once a NaN got into the pose it
     * stayed there for good.)</li>
     * <li>A module whose distance changed by more than {@link #MAX_MODULE_STEP_METERS} gets
     * weight 0 for this update (its new reading still becomes the baseline), as long as at
     * least two modules are left. A rebooted encoder used to add its whole odometer to the
     * pose (about 1 m).</li>
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
        if (!isFinite(gyroAngle, modulePositions)) {
            return new Twist2d();
        }
        updateWeights(modulePositions);
        if (lastModulePositions == null) {
            lastModulePositions = new SwerveModulePosition[modulePositions.length];
        } else {
            for (int i = 0; i < modulePositions.length; i++) {
                moduleDisplacements[i] = calculateModuleDisplacement(lastModulePositions[i], modulePositions[i]);
            }
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

    /** @return Whether the gyro angle and every module distance and angle are finite. */
    private static boolean isFinite(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        if (!Double.isFinite(gyroAngle.getRadians())) {
            return false;
        }
        for (SwerveModulePosition position : modulePositions) {
            if (position == null || !Double.isFinite(position.distanceMeters)
                    || !Double.isFinite(position.angle.getRadians())) {
                return false;
            }
        }
        return true;
    }

    /**
     * Equal weights, except that modules whose distance jumped by more than
     * {@link #MAX_MODULE_STEP_METERS} since the last update get 0 (if at least two are left).
     */
    private void updateWeights(SwerveModulePosition[] modulePositions) {
        int goodModules = 0;
        boolean[] good = new boolean[modulesWeights.length];
        for (int i = 0; i < modulesWeights.length; i++) {
            good[i] = lastModulePositions == null || Math.abs(modulePositions[i].distanceMeters
                    - lastModulePositions[i].distanceMeters) <= MAX_MODULE_STEP_METERS;
            if (good[i]) {
                goodModules++;
            }
        }
        if (goodModules < 2) {
            goodModules = modulesWeights.length;
            Arrays.fill(good, true);
        }
        for (int i = 0; i < modulesWeights.length; i++) {
            modulesWeights[i] = good[i] ? 1.0 / goodModules : 0.0;
        }
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
     * Moves the module baseline and the pose's position by the module motion since the last
     * update, ignoring the gyro (the heading is kept), e.g. for a sample measured while a
     * blocking gyro setter ran, whose gyro reading may be either the old or the new yaw. With
     * equal weights the rotation cancels out of the average (the centroid is the center).
     *
     * @param modulePositions The current swerve module positions.
     * @return The robot-relative translation of this update (meters).
     */
    public Translation2d updateModulesOnly(SwerveModulePosition[] modulePositions) {
        if (!isFinite(Rotation2d.kZero, modulePositions)) {
            return Translation2d.kZero;
        }
        if (lastModulePositions == null) {
            setModulePositions(modulePositions);
            return Translation2d.kZero;
        }
        updateWeights(modulePositions);
        for (int i = 0; i < modulePositions.length; i++) {
            moduleDisplacements[i] = calculateModuleDisplacement(lastModulePositions[i], modulePositions[i]);
            lastModulePositions[i] = modulePositions[i].copy();
        }
        Translation2d displacement = calculateRobotDisplacement(moduleDisplacements, 0);
        pose = new Pose2d(pose.getTranslation().plus(displacement.rotateBy(pose.getRotation())), pose.getRotation());
        return displacement;
    }

    /**
     * Takes these module readings as the baseline for the next update without moving the pose,
     * e.g. for a sample measured before a reset (its motion happened before the reset pose).
     *
     * @param modulePositions Module readings, same order as the module locations.
     */
    public void setModulePositions(SwerveModulePosition[] modulePositions) {
        if (lastModulePositions == null) {
            lastModulePositions = new SwerveModulePosition[modulePositions.length];
        }
        for (int i = 0; i < modulePositions.length; i++) {
            lastModulePositions[i] = modulePositions[i].copy();
        }
    }

    /**
     * Resets the pose. The gyro reading at the moment of the reset is needed so the new
     * heading is kept on the next update instead of being replaced by the raw gyro.
     *
     * @param newPose   The new field pose.
     * @param gyroAngle The raw gyro reading that corresponds to newPose's heading.
     */
    public void resetPose(Pose2d newPose, Rotation2d gyroAngle) {
        this.pose = newPose;
        this.gyroOffset = newPose.getRotation().minus(gyroAngle);
    }

    /** @return The pure-odometry pose (no vision corrections). */
    public Pose2d getOdometryPose() {
        return this.pose;
    }
}
