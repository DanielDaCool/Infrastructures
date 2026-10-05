package frc.robot.RobotPose.Estimation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;

class DemaciaPoseEstimatorTest {
    private static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3) };

    private DemaciaPoseEstimator estimator;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
        estimator = new DemaciaPoseEstimator(MODULE_LOCATIONS, VecBuilder.fill(1, 1, 0));
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    /** All modules pointing forward, having driven {@code distance} meters. */
    private static OdometryData straightSample(double timestamp, double distance) {
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModulePosition(distance, Rotation2d.kZero);
        }
        return new OdometryData(timestamp, Rotation2d.kZero, modules);
    }

    @Test
    void firstSampleDoesNotMoveTheRobot() {
        estimator.addOdometryData(List.of(straightSample(0, 7.0)));
        assertEquals(0, estimator.getEstimatedPose().getX(), 1e-9);
    }

    @Test
    void fiveSamplesPerLoopAddUpToTheDistanceDriven() {
        estimator.addOdometryData(List.of(straightSample(0, 0)));
        double distance = 0;
        // 1 s at 2 m/s, 250 Hz samples handed over 5 at a time (one 20 ms loop).
        for (int loop = 0; loop < 50; loop++) {
            List<OdometryData> samples = new ArrayList<>();
            for (int i = 0; i < 5; i++) {
                SimHooks.stepTiming(0.004);
                distance += 2 * 0.004;
                samples.add(straightSample(Timer.getFPGATimestamp(), distance));
            }
            estimator.addOdometryData(samples);
        }
        assertEquals(2.0, estimator.getEstimatedPose().getX(), 1e-9);
        assertEquals(0, estimator.getEstimatedPose().getY(), 1e-9);
    }

    @Test
    void samplesWithTheSameOrOlderTimestampAreNotLost() {
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0)));
        estimator.addOdometryData(List.of(
            straightSample(1.0, 0.1),   // same timestamp as the previous sample
            straightSample(0.99, 0.2),  // older than the previous sample
            straightSample(1.01, 0.3)));
        assertEquals(0.3, estimator.getEstimatedPose().getX(), 1e-9);
    }

    @Test
    void visionSlightlyNewerThanTheLatestSampleIsUsed() {
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0), straightSample(1.004, 0.01)));
        // A frame 3 ms newer than the newest sample, trusted exactly on x and y.
        estimator.addVisionMeasurement(new Pose2d(5, 0, Rotation2d.kZero), 1.007,
            VecBuilder.fill(0, 0, Double.POSITIVE_INFINITY));
        assertEquals(5.0, estimator.getEstimatedPose().getX(), 1e-9);
    }

    @Test
    void visionFarAheadOfTheLatestSampleIsDropped() {
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0), straightSample(1.004, 0.01)));
        estimator.addVisionMeasurement(new Pose2d(5, 0, Rotation2d.kZero), 1.2,
            VecBuilder.fill(0, 0, Double.POSITIVE_INFINITY));
        assertEquals(0.01, estimator.getEstimatedPose().getX(), 1e-9);
    }
}
