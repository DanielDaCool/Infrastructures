package frc.robot.RobotPose;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.RobotPose.Vision.BaseVisionSource;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionConfig;
import frc.robot.RobotPose.Vision.VisionSource;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera2d;
import frc.robot.RobotPose.Vision.VisionTypes.LimelightTagCamera3d;
import frc.robot.RobotPose.Vision.VisionTypes.Quest;
import frc.robot.RobotPose.Vision.visionConfigs.LimelightTagCamera2dConfig;
import frc.robot.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;
import frc.robot.RobotPose.Vision.visionConfigs.QuestConfig;

/**
 * The whole loop: {@code RobotPose.initialize} and {@code periodic()} every 20 ms of simulated
 * time, with odometry from a known path, the real MegaTag2, 2D Limelight and Quest classes behind
 * fake devices (see {@link LoopSim}), and a fake source for hand-made frames.
 */
class RobotPoseLoopTest {
    private static final Pose2d START = new Pose2d(7, 3, Rotation2d.kZero);
    private static final Transform3d FRONT_3D_OFFSET = new Transform3d(0.3, 0, 0.3, new Rotation3d(0, Math.toRadians(-15), 0));
    /** Back of the robot, facing backward, tilted up 20 deg (2D convention: positive = up). */
    private static final Transform3d BACK_2D_OFFSET = new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(20), Math.PI));
    private static final Transform3d QUEST_OFFSET = new Transform3d(0.1, 0.05, 0.5, new Rotation3d());
    /** Tag behind the robot at the start (2026 field). */
    private static final int BACK_TAG = 26;
    private static int cameraCount = 0;

    private LoopSim sim;
    private LoopSim.FakeLimelight3d megaTag2Device;
    private LoopSim.FakeLimelight2d limelight2dDevice;
    private LoopSim.FakeQuestNav questNav;
    private LimelightTagCamera3d megaTag2;
    private LimelightTagCamera2d limelight2d;
    private Quest quest;
    private LoopSim.FakeSource fakeSource;

    /** Sets up the robot at {@code START} (the estimate reset to {@code believedStart}) with every source. */
    private void start(LoopSim.Path path, Pose2d believedStart, boolean highRateOdometry) {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
        SimHooks.stepTiming(1.0);
        sim = new LoopSim(path, START);
        sim.highRateOdometry = highRateOdometry;
        // NetworkTables keeps values between tests, so every test gets its own camera names.
        cameraCount++;
        String name3d = "front" + cameraCount;
        String name2d = "back" + cameraCount;
        megaTag2Device = sim.new FakeLimelight3d(name3d);
        limelight2dDevice = sim.new FakeLimelight2d(name2d, BACK_2D_OFFSET, BACK_TAG);
        questNav = sim.new FakeQuestNav(QUEST_OFFSET);
        VisionConfig visionConfig = new VisionConfig(
                new LimelightTagCamera3dConfig(name3d, FRONT_3D_OFFSET, VecBuilder.fill(0.02, 0.02, Double.POSITIVE_INFINITY)),
                new LimelightTagCamera2dConfig(name2d, BACK_2D_OFFSET, VecBuilder.fill(0.02, 0.02, Double.POSITIVE_INFINITY)));
        megaTag2 = (LimelightTagCamera3d) visionConfig.getSources().get(0);
        limelight2d = (LimelightTagCamera2d) visionConfig.getSources().get(1);
        quest = new Quest(new QuestConfig("quest" + cameraCount, QUEST_OFFSET,
                VecBuilder.fill(0.03, 0.03, Double.POSITIVE_INFINITY)), questNav);
        visionConfig.getSources().add(quest);
        fakeSource = new LoopSim.FakeSource();
        visionConfig.getSources().add(fakeSource);
        RobotPose.initialize(sim::takeSamples,
                angle -> {
                    sim.setYawWrites.add(angle);
                    sim.writeGyro(angle);
                },
                angle -> {
                    sim.poseResetGyroWrites.add(angle);
                    sim.writeGyro(angle);
                },
                LoopSim.MODULE_LOCATIONS, VecBuilder.fill(0.05, 0.05, 0), visionConfig);
        RobotPose.getInstance().resetPose(believedStart);
    }

    private void start(LoopSim.Path path) {
        start(path, START, true);
    }

    private static RobotPose robotPose() {
        return RobotPose.getInstance();
    }

    private static String cm(double[] error) {
        return String.format("rms %.1f cm, max %.1f cm", error[0] * 100, error[1] * 100);
    }

    @AfterEach
    void tearDown() {
        BaseVisionSource.setOnCreate(source -> { });
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    @Test
    void followsTheTruthWithEverySourceAt250Hz() {
        start(LoopSim.DRIVE_CURVE_SPIN);
        sim.run(8);
        double[] error = sim.error(0.5, 8);
        System.out.println("drive/curve/spin, 250 Hz, all sources: " + cm(error)
                + ", 2D frames published " + limelight2dDevice.published);
        assertTrue(error[0] < 0.04, cm(error));
        assertTrue(error[1] < 0.12, cm(error));
        assertTrue(sim.headingError(0, 8) < 1e-6);
        assertTrue(limelight2dDevice.published > 0);
    }

    @Test
    void followsTheTruthWithOneSamplePerLoop() {
        start(LoopSim.DRIVE_CURVE_SPIN, START, false);
        sim.run(8);
        double[] error = sim.error(0.5, 8);
        System.out.println("drive/curve/spin, 50 Hz, all sources: " + cm(error));
        assertTrue(error[0] < 0.05, cm(error));
        assertTrue(error[1] < 0.15, cm(error));
    }

    @Test
    void wheelOdometryAloneDrifts() {
        // Reference for the numbers above: no vision at all, 2 % wheel error.
        start(LoopSim.DRIVE_CURVE_SPIN);
        megaTag2Device.tagCount = 0;
        sim.limelight2ds.clear();
        questNav.connected = false;
        sim.run(8);
        double[] error = sim.error(0.5, 8);
        System.out.println("drive/curve/spin, odometry only: " + cm(error));
        assertTrue(error[1] > 0.1, cm(error));
    }

    @Test
    void megaTag2FramesAreDroppedWhileSpinningFast() {
        start(LoopSim.DRIVE_CURVE_SPIN);
        int usedWhileDriving = 0;
        while (sim.time() < 3) {
            sim.step();
            usedWhileDriving += megaTag2.getPoseEstimates().size();
        }
        sim.run(1.3);
        int usedWhileSpinning = 0;
        while (sim.time() < 4.75) {
            sim.step();
            usedWhileSpinning += megaTag2.getPoseEstimates().size();
        }
        assertTrue(usedWhileDriving > 50, "used while driving: " + usedWhileDriving);
        assertEquals(0, usedWhileSpinning);
        double[] error = sim.error(4, 5);
        System.out.println("spin at 4 rad/s: " + cm(error));
        assertTrue(error[1] < 0.1, cm(error));
    }

    @Test
    void megaTag2FramesSolvedWithAnotherHeadingAreDropped() {
        start(LoopSim.STRAIGHT);
        megaTag2Device.headingErrorRadians = Math.toRadians(10);
        int used = 0;
        for (int i = 0; i < 100; i++) {
            sim.step();
            used += megaTag2.getPoseEstimates().size();
        }
        assertEquals(0, used);
        double[] error = sim.error(0.5, 2);
        System.out.println("MegaTag2 with a 10 deg heading error (dropped): " + cm(error));
        assertTrue(error[1] < 0.1, cm(error));
    }

    @Test
    void aQuestJumpIsIgnoredThenRealigned() {
        start(LoopSim.DRIVE_CURVE_SPIN);
        sim.run(2);
        questNav.jump = new Translation2d(0.5, -0.3);
        int usedDuringBlackout = 0;
        while (sim.time() < 2.45) {
            sim.step();
            usedDuringBlackout += quest.getPoseEstimates().size();
        }
        sim.run(0.4);
        double worstQuestError = 0;
        int usedAfter = 0;
        while (sim.time() < 4) {
            sim.step();
            for (TimestampedVisionMeasurement measurement : quest.getPoseEstimates()) {
                usedAfter++;
                worstQuestError = Math.max(worstQuestError, measurement.pose().getTranslation()
                        .getDistance(sim.truthAt(measurement.timestampSeconds()).getTranslation()));
            }
        }
        double[] error = sim.error(2, 4);
        System.out.println("Quest jump 0.58 m: " + cm(error) + ", re-aligned Quest frames off by "
                + String.format("%.1f cm", worstQuestError * 100));
        assertEquals(0, usedDuringBlackout);
        assertTrue(usedAfter > 5);
        assertTrue(worstQuestError < 0.05, "quest error " + worstQuestError);
        assertTrue(error[1] < 0.1, cm(error));
    }

    @Test
    void aShortQuestDisconnectDoesNotReanchorButALongOneDoes() {
        start(LoopSim.STRAIGHT);
        sim.run(1);
        int anchors = questNav.setPoseCalls;
        questNav.connected = false;
        sim.run(0.2);
        questNav.connected = true;
        sim.run(0.5);
        assertEquals(anchors, questNav.setPoseCalls);
        questNav.connected = false;
        sim.run(1.0);
        questNav.connected = true;
        sim.run(0.1);
        assertEquals(anchors + 1, questNav.setPoseCalls);
        double[] error = sim.error(0.5, sim.time());
        System.out.println("Quest disconnects: " + cm(error));
        assertTrue(error[1] < 0.1, cm(error));
    }

    @Test
    void anOutlierFrameIsRejected() {
        start(LoopSim.STILL);
        sim.run(1);
        Pose2d before = robotPose().getEstimatedPose();
        fakeSource.add(sim.truth.plus(new Transform2d(2, 0, Rotation2d.kZero)), Timer.getFPGATimestamp() - 0.01, 0.02);
        sim.step();
        double moved = robotPose().getEstimatedPose().getTranslation().getDistance(before.getTranslation());
        System.out.printf("outlier 2 m off: estimate moved %.2f cm%n", moved * 100);
        assertTrue(moved < 0.01);
    }

    @Test
    void aWrongStartPoseIsEscaped() {
        start(LoopSim.DRIVE_CURVE_SPIN, START.plus(new Transform2d(1.0, 0.6, Rotation2d.kZero)), true);
        double recovered = Double.NaN;
        // The Quest was anchored to the wrong pose too: once the cameras win, it must be re-aligned.
        double worstQuestError = 0;
        int questUsedAtTheEnd = 0;
        while (sim.time() < 8) {
            sim.step();
            double[] last = sim.errors.get(sim.errors.size() - 1);
            if (Double.isNaN(recovered) && last[1] < 0.1) {
                recovered = last[0];
            }
            if (sim.time() > 6) {
                for (TimestampedVisionMeasurement measurement : quest.getPoseEstimates()) {
                    questUsedAtTheEnd++;
                    worstQuestError = Math.max(worstQuestError, measurement.pose().getTranslation()
                            .getDistance(sim.truthAt(measurement.timestampSeconds()).getTranslation()));
                }
            }
        }
        double[] error = sim.error(recovered + 1, 8);
        System.out.printf("start pose 1.17 m off: under 10 cm after %.2f s, then %s; Quest frames at the end off by %.1f cm%n",
                recovered, cm(error), worstQuestError * 100);
        assertTrue(recovered < 2.0, "recovered at " + recovered);
        assertTrue(questUsedAtTheEnd > 5);
    }

    @Test
    void aRobotCarriedWhileDisabledIsFollowed() {
        start(LoopSim.STILL);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        // Carried 1.1 m between 0.5 s and 2 s: the wheels don't turn.
        sim.externalVelocity = LoopSim.carry(0.5, 2.0, new Translation2d(1.0, 0.5));
        sim.run(3);
        double atEnable = sim.errors.get(sim.errors.size() - 1)[1];
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        sim.run(1.0);
        double[] after = sim.error(sim.time() - 1, sim.time());
        System.out.printf("carried 1.1 m while disabled: peak %s, %.1f cm at enable, then %s%n",
                cm(sim.error(0, 3)), atEnable * 100, cm(after));
        assertTrue(atEnable < 0.05, "at enable " + atEnable);
        assertTrue(after[1] < 0.05, cm(after));
    }

    @Test
    void theFuturePoseUsesTheWheelVelocity() {
        start(LoopSim.STRAIGHT);
        sim.run(2);
        Pose2d now = robotPose().getEstimatedPose();
        Pose2d future = robotPose().getFuturePose(0.5);
        // 2 m/s for 0.5 s; the wheels read 2 % more.
        assertEquals(1.0 * sim.wheelScale, future.getX() - now.getX(), 0.02);
        assertEquals(0, future.getY() - now.getY(), 0.02);
        assertEquals(now.getRotation().getRadians(), future.getRotation().getRadians(), 1e-12);
    }

    @Test
    void theRegistrationHookSeesEverySource() {
        List<VisionSource> registered = new ArrayList<>();
        BaseVisionSource.setOnCreate(registered::add);
        start(LoopSim.STILL);
        assertTrue(registered.contains(megaTag2));
        assertTrue(registered.contains(limelight2d));
        assertTrue(registered.contains(quest));
    }

    @Test
    void aNaNOdometrySampleIsSurvived() {
        start(LoopSim.DRIVE_CURVE_SPIN);
        sim.run(2);
        sim.nanNextSample = true;
        sim.run(1);
        Pose2d pose = robotPose().getEstimatedPose();
        assertTrue(Double.isFinite(pose.getX()) && Double.isFinite(pose.getY()));
        double[] error = sim.error(2, 3);
        System.out.println("NaN module sample at 2 s: " + cm(error));
        assertTrue(error[1] < 0.1, cm(error));
    }

    @Test
    void resetPoseAndSetYawWriteTheGyroAndDropOlderFrames() {
        start(LoopSim.DRIVE_CURVE_SPIN);
        sim.run(2);
        assertEquals(1, sim.poseResetGyroWrites.size());  // the reset at the start
        double beforeReset = Timer.getFPGATimestamp() - 0.03;
        Pose2d target = sim.truth;
        robotPose().resetPose(target);
        assertEquals(2, sim.poseResetGyroWrites.size());
        assertEquals(target.getRotation().getRadians(), sim.poseResetGyroWrites.get(1).getRadians(), 1e-12);
        assertEquals(0, robotPose().getEstimatedPose().getTranslation().getDistance(target.getTranslation()), 1e-12);
        // A frame captured before the reset, 0.3 m off, arrives after it.
        fakeSource.add(sim.truthAt(beforeReset).plus(new Transform2d(0.3, 0, Rotation2d.kZero)), beforeReset, 0.01);
        sim.step();
        assertTrue(robotPose().getEstimatedPose().getTranslation().getDistance(sim.truth.getTranslation()) < 0.03);

        Rotation2d heading = sim.truth.getRotation();
        robotPose().setYaw(heading);
        assertEquals(1, sim.setYawWrites.size());
        assertEquals(heading.getRadians(), sim.setYawWrites.get(0).getRadians(), 1e-12);
        assertEquals(heading.getRadians(), sim.gyroAngle().getRadians(), 1e-12);
        sim.run(2);
        double[] error = sim.error(2, 4);
        System.out.println("resetPose + setYaw while driving: " + cm(error));
        assertTrue(error[1] < 0.1, cm(error));
        assertTrue(sim.headingError(2, 4) < 1e-6);
    }

    @Test
    void aShoveWhileStandingIsRecovered() {
        start(LoopSim.STILL);
        sim.run(1);
        // Shoved 0.5 m with the wheels locked.
        sim.externalVelocity = LoopSim.shove(1.0, new Translation2d(2.0, 0));
        double recovered = Double.NaN;
        while (sim.time() < 4) {
            sim.step();
            double[] last = sim.errors.get(sim.errors.size() - 1);
            if (Double.isNaN(recovered) && last[0] > 1.5 && last[1] < 0.05) {
                recovered = last[0] - 1.0;
            }
        }
        System.out.printf("shoved 0.5 m: under 5 cm %.2f s after the shove, %s%n", recovered, cm(sim.error(1, 4)));
        assertFalse(Double.isNaN(recovered));
        assertTrue(sim.error(3, 4)[1] < 0.05);
    }

    @Test
    void drivingOverABumpWithTheTilt() {
        double[] withAndWithout = new double[2];
        for (int run = 0; run < 2; run++) {
            start(LoopSim.STRAIGHT);
            // Vision off, so only the odometry is measured.
            megaTag2Device.tagCount = 0;
            sim.limelight2ds.clear();
            questNav.connected = false;
            sim.wheelScale = 1.0;
            // 1.0-1.4 s: tilted up to 20 deg, the wheels count 60 % more (climb and spin).
            sim.tiltAt = t -> Math.toRadians(20) * Math.max(0, Math.sin(Math.PI * (t - 1.0) / 0.4)) * (t >= 1.0 && t <= 1.4 ? 1 : 0);
            sim.bumpExtraDistance = t -> t >= 1.0 && t <= 1.4 ? 0.6 : 0.0;
            sim.reportTilt = run == 0;
            sim.run(2);
            withAndWithout[run] = sim.errors.get(sim.errors.size() - 1)[1];
        }
        System.out.printf("bump, odometry only: %.1f cm with the tilt, %.1f cm without%n",
                withAndWithout[0] * 100, withAndWithout[1] * 100);
        assertTrue(withAndWithout[0] < withAndWithout[1]);
        assertTrue(sim.headingError(0, 2) < 1e-6);
    }
}
