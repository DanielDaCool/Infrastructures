package frc.robot.RobotPose.Estimation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.MeasurementKind;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.VisionResult;

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

    /** All modules pointing {@code wheelAngle} (robot relative), having driven {@code distance} meters. */
    private static OdometryData sample(double timestamp, double distance, Rotation2d wheelAngle) {
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModulePosition(distance, wheelAngle);
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
        estimator.addVisionMeasurement(new Pose2d(0.2, 0, Rotation2d.kZero), 1.007,
            VecBuilder.fill(0, 0, Double.POSITIVE_INFINITY));
        assertEquals(0.2, estimator.getEstimatedPose().getX(), 1e-9);
    }

    @Test
    void visionFarAheadOfTheLatestSampleIsDropped() {
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0), straightSample(1.004, 0.01)));
        assertEquals(VisionResult.DROPPED, estimator.addVisionMeasurement(new Pose2d(0.2, 0, Rotation2d.kZero), 1.2,
            VecBuilder.fill(0, 0, Double.POSITIVE_INFINITY)));
        assertEquals(0.01, estimator.getEstimatedPose().getX(), 1e-9);
    }

    /** A hit from the front, over {@link DemaciaPoseEstimator#COLLISION_ACCELERATION} (6 g, backward). */
    private static final Translation2d HIT_FROM_THE_FRONT = new Translation2d(-60, 0);

    @Test
    void motionIntoAHitIsDropped() {
        estimator.setAdaptiveTrust(false);
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0)));
        SimHooks.stepTiming(0.02);
        // Driving forward into a wall: the hit pushes the robot backward (-x).
        estimator.addOdometryData(List.of(straightSample(1.02, 0.1)), HIT_FROM_THE_FRONT);
        assertEquals(0, estimator.getEstimatedPose().getX(), 1e-9);
        assertEquals(true, estimator.isColliding());
    }

    @Test
    void motionToTheSideOfAHitIsKept() {
        estimator.setAdaptiveTrust(false);
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(sample(1.0, 0, Rotation2d.kCCW_Pi_2)));
        SimHooks.stepTiming(0.02);
        // Driving left (+y) while hit from the front: the motion is perpendicular to the hit.
        estimator.addOdometryData(List.of(sample(1.02, 0.1, Rotation2d.kCCW_Pi_2)), HIT_FROM_THE_FRONT);
        assertEquals(0.1, estimator.getEstimatedPose().getY(), 1e-9);
    }

    @Test
    void collisionEndsAfterTheHoldTime() {
        estimator.setAdaptiveTrust(false);
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0)));
        SimHooks.stepTiming(0.02);
        estimator.addOdometryData(List.of(straightSample(1.02, 0.1)), HIT_FROM_THE_FRONT);
        SimHooks.stepTiming(DemaciaPoseEstimator.COLLISION_HOLD_SECONDS + 0.01);
        estimator.addOdometryData(List.of(straightSample(Timer.getFPGATimestamp(), 0.2)), Translation2d.kZero);
        assertEquals(false, estimator.isColliding());
        assertEquals(0.1, estimator.getEstimatedPose().getX(), 1e-9);
    }

    @Test
    void accelerationBelowTheThresholdIsNotAHit() {
        estimator.setAdaptiveTrust(false);
        SimHooks.stepTiming(1.0);
        estimator.addOdometryData(List.of(straightSample(1.0, 0)));
        SimHooks.stepTiming(0.02);
        // 4 g: a hit for the old 3 g threshold, not for 5 g.
        estimator.addOdometryData(List.of(straightSample(1.02, 0.1)), new Translation2d(-4 * 9.81, 0));
        assertEquals(0.1, estimator.getEstimatedPose().getX(), 1e-9);
    }

    private static Matrix<N3, N1> xyStd(double std) {
        return VecBuilder.fill(std, std, Double.POSITIVE_INFINITY);
    }

    /** All modules forward at {@code distance}, gyro at {@code gyro} radians. */
    private static OdometryData sample(double timestamp, double distance, double gyro) {
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModulePosition(distance, Rotation2d.kZero);
        }
        return new OdometryData(timestamp, new Rotation2d(gyro), modules);
    }

    /** Resets to the origin at FPGA time 1 and adds a standing sample at that time. */
    private void resetAtOneSecond() {
        SimHooks.stepTiming(1.0);
        estimator.resetPose(Pose2d.kZero, Rotation2d.kZero);
        estimator.addOdometryData(List.of(straightSample(1.0, 0)));
    }

    /** One 20 ms loop standing still at {@code distance}; returns the sample time. */
    private double standingLoop(double distance, Translation2d acceleration) {
        SimHooks.stepTiming(0.02);
        double now = Timer.getFPGATimestamp();
        estimator.addOdometryData(List.of(straightSample(now, distance)), acceleration);
        return now;
    }

    private double variance() {
        return Math.pow(estimator.getPositionStdDev(), 2);
    }

    @Test
    void uncertaintyGrowsWithDistanceAndTimeNotWithTheSampleRate() {
        for (int hz : new int[] {50, 250}) {
            SimHooks.restartTiming();
            estimator = new DemaciaPoseEstimator(MODULE_LOCATIONS, VecBuilder.fill(1, 1, 0));
            resetAtOneSecond();
            // 2 m in 1 s.
            for (int i = 1; i <= hz; i++) {
                estimator.addOdometryData(List.of(straightSample(1.0 + (double) i / hz, 2.0 * i / hz)));
            }
            double expected = DemaciaPoseEstimator.RESET_VARIANCE + DemaciaPoseEstimator.VARIANCE_PER_METER * 2
                    + DemaciaPoseEstimator.VARIANCE_PER_SECOND * 1;
            assertEquals(expected, variance(), 1e-9);
        }
    }

    @Test
    void visionPullsByPOverPPlusR() {
        resetAtOneSecond();
        double t = standingLoop(0, Translation2d.kZero);
        double p = variance();
        double r = 0.1 * 0.1;
        double k = p / (p + r);
        assertEquals(VisionResult.ACCEPTED, estimator.addVisionMeasurement(new Pose2d(0.1, 0, Rotation2d.kZero), t, xyStd(0.1)));
        assertEquals(0.1 * k, estimator.getEstimatedPose().getX(), 1e-12);
        assertEquals((1 - k) * p, variance(), 1e-12);
    }

    @Test
    void aLateFrameUsesTheUncertaintyAtItsTime() {
        resetAtOneSecond();
        estimator.addOdometryData(List.of(straightSample(1.5, 1.0), straightSample(2.0, 2.0)));
        double addedSince = DemaciaPoseEstimator.VARIANCE_PER_METER * 1.0 + DemaciaPoseEstimator.VARIANCE_PER_SECOND * 0.5;
        double pNow = variance();
        double pAtFrame = pNow - addedSince;
        double k = pAtFrame / (pAtFrame + 0.01);
        // The estimate was at (1, 0) at t = 1.5; the frame says y = 0.1 there.
        estimator.addVisionMeasurement(new Pose2d(1.0, 0.1, Rotation2d.kZero), 1.5, xyStd(0.1));
        assertEquals(0.1 * k, estimator.getEstimatedPose().getY(), 1e-12);
        assertEquals(2.0, estimator.getEstimatedPose().getX(), 1e-12);
        assertEquals((1 - k) * pAtFrame + addedSince, variance(), 1e-12);
    }

    @Test
    void aJumpIsRejectedUntilEnoughFramesAgree() {
        resetAtOneSecond();
        for (int i = 1; i <= DemaciaPoseEstimator.ESCAPE_REJECTED_FRAMES; i++) {
            double t = standingLoop(0, Translation2d.kZero);
            VisionResult result = estimator.addVisionMeasurement(new Pose2d(2.0, 0, Rotation2d.kZero), t, xyStd(0.05));
            if (i < DemaciaPoseEstimator.ESCAPE_REJECTED_FRAMES) {
                assertEquals(VisionResult.REJECTED, result);
                assertEquals(0, estimator.getEstimatedPose().getX(), 1e-12);
            } else {
                assertEquals(VisionResult.ACCEPTED, result);
            }
        }
        // P was raised to the 2 m error, so the frame is taken almost fully.
        assertEquals(2.0 * 4 / (4 + 0.05 * 0.05), estimator.getEstimatedPose().getX(), 1e-9);
        assertEquals(1, estimator.getEscapes());
        assertEquals(DemaciaPoseEstimator.ESCAPE_REJECTED_FRAMES, estimator.getRejectedFrames());
    }

    @Test
    void rejectedFramesThatDisagreeDoNotEscape() {
        resetAtOneSecond();
        for (int i = 0; i < 2 * DemaciaPoseEstimator.ESCAPE_WINDOW_FRAMES; i++) {
            double t = standingLoop(0, Translation2d.kZero);
            double y = i % 2 == 0 ? 0 : 1.0;  // 1 m apart
            assertEquals(VisionResult.REJECTED,
                    estimator.addVisionMeasurement(new Pose2d(2.0, y, Rotation2d.kZero), t, xyStd(0.05)));
        }
        assertEquals(0, estimator.getEstimatedPose().getX(), 1e-12);
    }

    @Test
    void questFramesThatEscapeRealignTheQuest() {
        resetAtOneSecond();
        VisionResult result = null;
        for (int i = 1; i <= DemaciaPoseEstimator.ESCAPE_REJECTED_FRAMES; i++) {
            double t = standingLoop(0, Translation2d.kZero);
            result = estimator.addVisionMeasurement(new Pose2d(2.0, 1.0, Rotation2d.kZero), t, xyStd(0.05),
                    MeasurementKind.QUEST);
        }
        assertEquals(VisionResult.REALIGN_QUEST, result);
        // The Quest frames are 2 m / 1 m off, so they have to move back by that.
        assertEquals(-2.0, estimator.getQuestRealignment().getX(), 1e-12);
        assertEquals(-1.0, estimator.getQuestRealignment().getY(), 1e-12);
        assertEquals(0, estimator.getEstimatedPose().getX(), 1e-12);
    }

    @Test
    void theJumpFilterIsOffWhileDisabled() {
        resetAtOneSecond();
        estimator.setEnabled(false);
        double t = standingLoop(0, Translation2d.kZero);
        assertEquals(VisionResult.ACCEPTED,
                estimator.addVisionMeasurement(new Pose2d(3.0, 0, Rotation2d.kZero), t, xyStd(0.05)));
        assertTrue(estimator.getEstimatedPose().getX() > 2.0);
    }

    @Test
    void badFramesAreDropped() {
        estimator.setFieldSize(16.5, 8.1);
        resetAtOneSecond();
        double t = standingLoop(0, Translation2d.kZero);
        assertEquals(VisionResult.DROPPED,
                estimator.addVisionMeasurement(new Pose2d(Double.NaN, 0, Rotation2d.kZero), t, xyStd(0.05)));
        assertEquals(VisionResult.DROPPED,
                estimator.addVisionMeasurement(new Pose2d(0, 0, Rotation2d.kZero), t, xyStd(Double.NaN)));
        assertEquals(VisionResult.DROPPED,
                estimator.addVisionMeasurement(new Pose2d(17.1, 0, Rotation2d.kZero), t, xyStd(0.05)));
        assertEquals(VisionResult.DROPPED,
                estimator.addVisionMeasurement(new Pose2d(0, -0.6, Rotation2d.kZero), t, xyStd(0.05)));
        // Older than the history.
        for (int i = 0; i < 100; i++) {
            standingLoop(0, Translation2d.kZero);
        }
        assertEquals(VisionResult.DROPPED, estimator.addVisionMeasurement(Pose2d.kZero, t, xyStd(0.05)));
        assertEquals(0, estimator.getEstimatedPose().getX(), 1e-12);
    }

    @Test
    void framesFromBeforeTheFirstSampleAfterAResetAreDropped() {
        resetAtOneSecond();
        standingLoop(0, Translation2d.kZero);
        SimHooks.stepTiming(0.02);
        double resetTime = Timer.getFPGATimestamp();
        estimator.resetPose(new Pose2d(1, 1, Rotation2d.kZero), Rotation2d.kZero);
        // Nothing after the reset yet.
        assertEquals(VisionResult.DROPPED,
                estimator.addVisionMeasurement(new Pose2d(1.01, 1, Rotation2d.kZero), resetTime, xyStd(0.05)));
        double t = standingLoop(0, Translation2d.kZero);
        assertEquals(VisionResult.DROPPED,
                estimator.addVisionMeasurement(new Pose2d(1.01, 1, Rotation2d.kZero), resetTime + 0.01, xyStd(0.05)));
        assertEquals(VisionResult.ACCEPTED,
                estimator.addVisionMeasurement(new Pose2d(1.01, 1, Rotation2d.kZero), t, xyStd(0.05)));
    }

    @Test
    void aResetSetsAModerateUncertainty() {
        resetAtOneSecond();
        estimator.addOdometryData(List.of(straightSample(2.0, 10.0)));
        assertTrue(estimator.getPositionStdDev() > 0.15);
        estimator.resetPose(Pose2d.kZero, Rotation2d.kZero);
        assertEquals(0.1, estimator.getPositionStdDev(), 1e-12);
    }

    @Test
    void withoutAdaptiveTrustTheGainIsFixed() {
        estimator.setAdaptiveTrust(false);
        resetAtOneSecond();
        estimator.addOdometryData(List.of(straightSample(2.0, 10.0)));
        assertEquals(1.0, estimator.getPositionStdDev(), 1e-12);
        // q = 1 (state std 1), r = 1: K = q / (q + sqrt(q r)) = 0.5.
        estimator.addVisionMeasurement(new Pose2d(10.0, 1.0, Rotation2d.kZero), 2.0, xyStd(1.0));
        assertEquals(0.5, estimator.getEstimatedPose().getY(), 1e-12);
        assertEquals(1.0, estimator.getPositionStdDev(), 1e-12);
    }

    @Test
    void beingPushedWhileTheWheelsStandStillIsSlip() {
        resetAtOneSecond();
        double before = variance();
        Translation2d push = new Translation2d(3.0, 0);
        // 0.5 s of a 3 m/s^2 shove that the wheels don't feel.
        for (int i = 0; i < 25; i++) {
            standingLoop(0, push);
        }
        assertTrue(estimator.isSlipping());
        assertTrue(variance() > before + 0.001);
    }

    @Test
    void noSlipWhenTheWheelsAndTheAccelerometerAgree() {
        resetAtOneSecond();
        double before = variance();
        for (int i = 0; i < 5; i++) {
            standingLoop(0, new Translation2d(0.01, 0));
        }
        assertFalse(estimator.isSlipping());
        assertEquals(before + 5 * 0.02 * DemaciaPoseEstimator.VARIANCE_PER_SECOND, variance(), 1e-9);
    }

    @Test
    void aHitAddsUncertaintyAndWheelsDrivingIntoItArePinned() {
        resetAtOneSecond();
        double distance = 0;
        Translation2d gentle = new Translation2d(0.01, 0);
        for (int i = 0; i < 3; i++) {
            distance += 0.04;  // 2 m/s
            standingLoop(distance, gentle);
        }
        double before = variance();
        distance += 0.04;
        standingLoop(distance, HIT_FROM_THE_FRONT);
        assertTrue(variance() >= before + DemaciaPoseEstimator.COLLISION_VARIANCE);
        // The wheels keep driving forward into the wall.
        distance += 0.04;
        standingLoop(distance, gentle);
        assertTrue(estimator.isPinned());
        assertTrue(estimator.getSecondsSinceHit() < 0.1);
    }

    @Test
    void outOfOrderFramesEndUpCloseToInOrder() {
        Pose2d[] results = new Pose2d[2];
        for (int order = 0; order < 2; order++) {
            SimHooks.restartTiming();
            estimator = new DemaciaPoseEstimator(MODULE_LOCATIONS, VecBuilder.fill(1, 1, 0));
            resetAtOneSecond();
            List<OdometryData> samples = new ArrayList<>();
            for (int i = 1; i <= 50; i++) {
                // 2 m/s while turning at 1 rad/s.
                samples.add(sample(1.0 + i * 0.02, 0.04 * i, 0.02 * i));
            }
            estimator.addOdometryData(samples);
            Pose2d a = new Pose2d(1.9, 0.6, Rotation2d.kZero);
            Pose2d b = new Pose2d(1.7, 0.5, Rotation2d.kZero);
            if (order == 0) {
                estimator.addVisionMeasurement(b, 1.95, xyStd(0.1));
                estimator.addVisionMeasurement(a, 2.0, xyStd(0.1));
            } else {
                estimator.addVisionMeasurement(a, 2.0, xyStd(0.1));
                estimator.addVisionMeasurement(b, 1.95, xyStd(0.1));
            }
            results[order] = estimator.getEstimatedPose();
        }
        assertEquals(0, results[0].getTranslation().getDistance(results[1].getTranslation()), 0.01);
    }

    @Test
    void theFuturePoseUsesTheOdometryVelocity() {
        resetAtOneSecond();
        double t = 1.0;
        for (int i = 1; i <= 10; i++) {
            t = 1.0 + i * 0.02;
            estimator.addOdometryData(List.of(straightSample(t, 0.04 * i)));
        }
        // A vision correction moves the pose, but not the velocity.
        estimator.addVisionMeasurement(new Pose2d(0.5, 0.1, Rotation2d.kZero), t, xyStd(0.1));
        Pose2d now = estimator.getEstimatedPose();
        Pose2d future = estimator.getFuturePose(0.5);
        assertEquals(now.getX() + 1.0, future.getX(), 1e-9);
        assertEquals(now.getY(), future.getY(), 1e-9);
        assertEquals(now.getX() + 2.0, estimator.getFuturePose(3.0).getX(), 1e-9);
    }

    @Test
    void thePeakYawRateIsTakenOverTheWindowBeforeTheFrame() {
        resetAtOneSecond();
        List<OdometryData> samples = new ArrayList<>();
        double gyro = 0;
        for (int i = 1; i <= 50; i++) {
            gyro += (i <= 10 ? 3.0 : 0.5) * 0.02;
            samples.add(sample(1.0 + i * 0.02, 0, gyro));
        }
        estimator.addOdometryData(samples);
        assertEquals(3.0, estimator.getPeakYawRate(1.2, 0.1), 1e-6);
        assertEquals(0.5, estimator.getPeakYawRate(2.0, 0.1), 1e-6);
    }

    @Test
    void thePoseAtAPastTimeIsTheEstimateMovedBackByTheOdometry() {
        resetAtOneSecond();
        estimator.addOdometryData(List.of(straightSample(1.5, 1.0), straightSample(2.0, 2.0)));
        estimator.addVisionMeasurement(new Pose2d(2.0, 0.1, Rotation2d.kZero), 2.0, xyStd(0.0));
        assertEquals(0.1, estimator.getEstimatedPose().getY(), 1e-12);
        Pose2d past = estimator.getPoseAt(1.25);
        // The first sample after the reset is 1 us late (kept in order), hence 1e-5.
        assertEquals(0.5, past.getX(), 1e-5);
        assertEquals(0.1, past.getY(), 1e-9);
    }
}
