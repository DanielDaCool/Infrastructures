package frc.robot.Path;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.SimClock;
import frc.robot.sim.SwerveSim;
import frc.robot.sim.SwerveSimConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/** FollowPathCommand on the WPILib HAL sim (DriverStationSim, CommandScheduler) driving a physics swerve with noisy sensors. */
class PathSimTest {
    /** Field-relative drive on the physics sim; pose from odometry on the measured (noisy, delayed) sensors. */
    static final class SimDrive extends SubsystemBase {
        final SwerveSim sim;
        final SwerveDriveKinematics kin;
        final SwerveDriveOdometry odo;
        final double freeSpeed;
        SwerveModuleState[] cmd;

        SimDrive(SwerveSimConfig cfg, SimClock clock, Pose2d start) {
            sim = new SwerveSim(cfg, clock);
            sim.resetPose(start);
            kin = new SwerveDriveKinematics(cfg.modulePositions);
            odo = new SwerveDriveOdometry(kin, sim.getGyroYaw(), sim.getMeasuredModulePositions(), start);
            freeSpeed = cfg.driveFreeSpeed();
            cmd = new SwerveModuleState[cfg.modulePositions.length];
            for (int i = 0; i < cmd.length; i++) cmd[i] = new SwerveModuleState();
        }

        Pose2d pose() { return odo.getPoseMeters(); }

        ChassisSpeeds fieldSpeeds() {
            return ChassisSpeeds.fromRobotRelativeSpeeds(kin.toChassisSpeeds(sim.getMeasuredModuleStates()), pose().getRotation());
        }

        void driveField(ChassisSpeeds field) {
            ChassisSpeeds robot = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(field, pose().getRotation()), 0.02);
            SwerveModuleState[] s = kin.toSwerveModuleStates(robot);
            SwerveDriveKinematics.desaturateWheelSpeeds(s, freeSpeed);
            SwerveModuleState[] measured = sim.getMeasuredModuleStates();
            for (int i = 0; i < s.length; i++) {
                if (s[i].speedMetersPerSecond == 0) s[i].angle = measured[i].angle; // hold the angle at rest
                s[i].optimize(measured[i].angle);
                s[i].cosineScale(measured[i].angle);
            }
            cmd = s;
        }

        void loop() {
            odo.update(sim.getGyroYaw(), sim.getMeasuredModulePositions());
            CommandScheduler.getInstance().run();
            sim.step(cmd);
        }
    }

    record Result(boolean finished, double time, double maxDist, double endErr, double headErrDeg, double moved) {}

    private static void alliance(AllianceStationID id) {
        DriverStationSim.setAllianceStationId(id);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    private static double distanceToPath(Path path, Translation2d p) {
        double best = Double.POSITIVE_INFINITY;
        for (Segment seg : path.segments())
            best = Math.min(best, seg.pointAt(MathUtil.clamp(seg.project(p), 0, seg.length())).getDistance(p));
        return best;
    }

    /** Runs the blue path on the given alliance from start (true pose); distances against the driven (maybe flipped) path. */
    private static Result run(String name, Path blue, AllianceStationID id, Pose2d start, double endDeg) {
        try (SimClock clock = SimClock.start()) {
            alliance(id);
            boolean red = id.name().startsWith("Red");
            Path driven = red ? blue.flipped() : blue;
            SimDrive drive = new SimDrive(SwerveSimConfig.defaults(), clock, start);
            FollowPathCommand cmd = new FollowPathCommand(blue, drive::pose, drive::fieldSpeeds, drive::driveField, drive);
            CommandScheduler.getInstance().schedule(cmd);
            double maxDist = 0, t0 = clock.now(), time = Double.NaN;
            for (int i = 0; i < 1000; i++) { // 20 s
                drive.loop();
                maxDist = Math.max(maxDist, distanceToPath(driven, drive.sim.getTruePose().getTranslation()));
                if (!cmd.isScheduled()) { time = clock.now() - t0; break; }
            }
            Pose2d end = drive.sim.getTruePose();
            double target = red && !PathConfig.MIRRORED_FIELD ? endDeg + 180 : red ? 180 - endDeg : endDeg;
            Result r = new Result(!Double.isNaN(time), time, maxDist, end.getTranslation().getDistance(driven.end()),
                    Math.toDegrees(MathUtil.angleModulus(Math.toRadians(target) - end.getRotation().getRadians())),
                    end.getTranslation().getDistance(start.getTranslation()));
            System.out.printf("SIM %-26s finished=%b t=%.2f s maxDist=%.3f m endErr=%.3f m headErr=%.2f deg progress=%.2f%n",
                    name, r.finished, r.time, r.maxDist, r.endErr, r.headErrDeg, cmd.progress());
            return r;
        } finally {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().unregisterAllSubsystems();
        }
    }

    private static void assertGood(Result r) {
        assertTrue(r.finished, "finished");
        assertTrue(r.maxDist < 0.10, "max distance to path " + r.maxDist);
        assertTrue(r.endErr < 0.06, "end error " + r.endErr); // odometry drift + POSITION_TOLERANCE
        assertTrue(Math.abs(r.headErrDeg) < 3, "heading error " + r.headErrDeg);
    }

    @AfterEach
    void reset() { DriverStationSim.resetData(); }

    @Test
    void leftAndRightCornersOnBlue() {
        assertGood(run("blue left corner", new Path(new Point(2, 2), new Point(5, 2), new Point(5, 5)),
                AllianceStationID.Blue1, new Pose2d(2, 2, Rotation2d.kZero), 0));
        assertGood(run("blue right corner", new Path(new Point(2, 5), new Point(5, 5), new Point(5, 2)),
                AllianceStationID.Blue1, new Pose2d(2, 5, Rotation2d.kZero), 0));
    }

    @Test
    void sCurveWithStopAndHeading() {
        Path path = new Path(new Point(2, 2), new Point(4, 2), new Point(4, 4).withRadius(0.5), new Point(6, 4).stop(0.5),
                new Point(6, 6).withAngle(Rotation2d.fromDegrees(90)));
        assertGood(run("blue S + stop + heading", path, AllianceStationID.Blue1, new Pose2d(2, 2, Rotation2d.kZero), 90));
    }

    @Test
    void redAllianceDrivesTheFlippedPath() {
        Path blue = new Path(new Point(2, 2), new Point(5, 2), new Point(5, 5).withAngle(Rotation2d.fromDegrees(45)));
        Translation2d s = blue.flipped().start();
        double startDeg = PathConfig.MIRRORED_FIELD ? 180 : 180; // blue heading 0 flipped
        assertGood(run("red left corner", blue, AllianceStationID.Red1, new Pose2d(s, Rotation2d.fromDegrees(startDeg)), 45));
    }

    @Test
    void startTooFarDoesNotDrive() {
        Path path = new Path(new Point(2, 2), new Point(5, 2));
        Result r = run("start 1 m away", path, AllianceStationID.Blue1, new Pose2d(2, 3, Rotation2d.kZero), 0);
        assertTrue(r.finished && r.time < 0.1, "ended at once");
        assertTrue(r.moved < 0.01, "moved " + r.moved);
    }
}
