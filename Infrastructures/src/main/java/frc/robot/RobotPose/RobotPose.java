package frc.robot.RobotPose;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionConfig;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionTypes.Quest;

/**
 * The robot's field pose. This is the only class the rest of the robot code should use
 * to read or reset the pose.
 *
 * <p>Every loop {@link #periodic()} feeds the {@link DemaciaPoseEstimator} with:
 * <ol>
 * <li>one odometry sample (gyro + swerve module positions) from the supplier passed to
 * {@link #initialize}, and</li>
 * <li>every new measurement from the {@link VisionSource}s in the {@link VisionConfig}.</li>
 * </ol>
 *
 * <p>Vision sources are handled generically through the {@link VisionSource} interface. The
 * only exception is {@link Quest}: the Quest reports poses relative to the pose it was last
 * set to, so RobotPose re-anchors it (see {@link Quest#setPose}) whenever it reconnects and
 * whenever the pose is reset.
 *
 * <p>Singleton: call {@link #initialize} once (in RobotContainer, after
 * the chassis is created), then {@link #getInstance()} everywhere else.
 */
public final class RobotPose {

    private static RobotPose instance;

    private final DemaciaPoseEstimator poseEstimator;
    /** Reads the gyro and module positions; called once per loop (and by {@link #getGyroAngle()}). */
    private final Supplier<OdometryData> odometryDataSupplier;
    /** Writes a new yaw to the gyro; used by {@link #setYaw}. */
    private final Consumer<Rotation2d> gyroYawSetter;
    /** Every configured vision source, including the Quest if there is one. */
    private final List<VisionSource> sources;

    private RobotPose(Supplier<OdometryData> odometryDataSupplier, Consumer<Rotation2d> gyroYawSetter,
        Translation2d[] moduleLocations, Matrix<N3, N1> stateStd, List<VisionSource> sources) {
        this.odometryDataSupplier = odometryDataSupplier;
        this.gyroYawSetter = gyroYawSetter;
        this.poseEstimator = new DemaciaPoseEstimator(odometryDataSupplier.get().swerveModules(), moduleLocations, stateStd);
        this.sources = sources;

        addLog();
    }

    private void addLog() {
        SmartDashboard.putData("chassis/reset gyro",
                new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset gyro 180",
                new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
    }

        /**
     * Runs one loop of pose estimation. Called from {@code Robot.robotPeriodic()}, after the
     * CommandScheduler.
     *
     * <ol>
     * <li>Adds this loop's odometry sample. This always happens first, so vision frames
     * captured up to now have odometry around them to be placed into.</li>
     * <li>Calls {@link VisionSource#periodic()} on every source. Sources read their device
     * here (and the 3D Limelight sends it the current heading).</li>
     * <li>For each source: if it is a Quest that just reconnected, re-anchors it to the
     * current estimate and skips it this loop; otherwise, if {@link VisionSource#shouldUpdate()}
     * is true, adds each of its measurements to the estimator.</li>
     * </ol>
     *
     * Steps 2 and 3 are separate loops so that every source has read its device before any
     * measurement is used.
     */
    public void periodic() {
        poseEstimator.addOdometryData(odometryDataSupplier.get());

        for (VisionSource source : sources) {
            source.periodic();
        }

        for (VisionSource source : sources) {
            if (source instanceof Quest && ((Quest) source).hasDrifted()) {
                ((Quest) source).setPose(poseEstimator.getEstimatedPose());
            }
            else if (source.shouldUpdate()) {
                for (TimestampedVisionMeasurement measurement : source.getPoseEstimates()) {
                    poseEstimator.addVisionMeasurement(measurement.pose(), measurement.timestampSeconds(),
                            measurement.stdDevs());
                }
            }
        }
    }

    /**
     * @return The latest fused field pose (blue-alliance origin, meters and radians), as of
     *         the last {@link #periodic()}.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPose();
    }

    /**
     * The estimated pose at a past FPGA timestamp (e.g. a vision frame's capture time). Only
     * the last {@link DemaciaPoseEstimator#HISTORY_LENGTH_SECONDS} are kept; older timestamps
     * return the oldest kept pose.
     *
     * @param timestampSeconds FPGA time, in seconds.
     */
    public Pose2d getEstimatedPoseAt(double timestampSeconds) {
        return poseEstimator.getPoseAt(timestampSeconds);
    }

    /**
     * Sets the robot's pose (position and heading). The gyro itself is not changed; the
     * estimator stores the difference between the gyro and the new heading instead. The
     * vision history is cleared and every Quest is re-anchored to the new pose.
     *
     * @param pose The new field pose.
     */
    public void resetPose(Pose2d pose) {
        resetEstimator(pose, getGyroAngle());
    }

    /**
     * Sets the heading only, keeping the current position. Unlike {@link #resetPose}, this
     * also writes the new angle to the gyro. Bound to the "chassis/reset gyro" and
     * "chassis/reset gyro 180" dashboard buttons.
     *
     * @param angle The new field heading; {@code null} does nothing.
     */
    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            gyroYawSetter.accept(angle);
            // The gyro was just set to `angle`, so pass it as the gyro reading (the new value may
            // not have been read back from the device yet).
            resetEstimator(new Pose2d(getEstimatedPose().getTranslation(), angle), angle);
        }
    }

    /**
     * Resets the estimator and re-anchors every Quest to the new pose. The Quest reports poses
     * relative to its last anchor, so without this its next frames pull the estimate back to
     * the old pose.
     */
    private void resetEstimator(Pose2d pose, Rotation2d gyroAngle) {
        poseEstimator.resetPose(pose, gyroAngle);
        for (VisionSource source : sources) {
            if (source instanceof Quest) {
                ((Quest) source).setPose(pose);
            }
        }
    }

    /**
     * @return The raw gyro reading, read now. This is not the field heading if the pose has
     *         been reset without setting the gyro; use {@link #getEstimatedPose()} for that.
     */
    public Rotation2d getGyroAngle() {
        return odometryDataSupplier.get().gyroAngle();
    }


    /**
     * Creates the singleton. Call once from RobotContainer, after the chassis is created.
     * A second call replaces the instance (and its history).
     *
     * @param odometryDataSupplier   Returns the current gyro angle and swerve module
     *                               positions; called every loop.
     * @param gyroYawSetter          Writes a new yaw to the gyro (used by {@link #setYaw}),
     *                               e.g. {@code angle -> gyro.setYaw(angle.getDegrees())}.
     * @param moduleLocations        Each module's position relative to the robot center
     *                               (meters), in the same order as the module positions.
     * @param stateStd               How much to trust odometry, per axis (x m, y m, theta rad).
     *                               Larger values let vision correct the pose faster. A 0 on
     *                               an axis means vision never changes that axis.
     * @param visionConfig           The vision sources to use. They are already created when
     *                               the config is built.
     */
    public static synchronized void initialize(Supplier<OdometryData> odometryDataSupplier,
            Consumer<Rotation2d> gyroYawSetter, Translation2d[] moduleLocations, Matrix<N3, N1> stateStd,
            VisionConfig visionConfig) {
        instance = new RobotPose(odometryDataSupplier, 
            gyroYawSetter,
            moduleLocations, 
            stateStd, 
            visionConfig.getSources());
    }

    /**
     * Returns the singleton instance, or null if initialize(...) has not been
     * called yet.
     */
    public static synchronized RobotPose getInstance() {
        return instance;
    }
}