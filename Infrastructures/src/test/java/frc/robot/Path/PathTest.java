package frc.robot.Path;

import static frc.robot.Path.PathConfig.*;
import static frc.robot.Path.PathFollower.MERGE_DISTANCE;
import static frc.robot.Path.PathFollower.MIN_SPEED;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class PathTest {
    private static final double DT = TimedRobot.kDefaultPeriod;

    private static Path left() { return new Path(new Point(0, 0), new Point(2, 0), new Point(2, 2)); }

    private static Path right() { return new Path(new Point(0, 0), new Point(2, 0), new Point(2, -2)); }

    private static Path sCurve() {
        return new Path(new Point(0, 0), new Point(2, 0), new Point(2, 2), new Point(4, 2));
    }

    private static Pose2d pose(double x, double y, double deg) { return new Pose2d(x, y, Rotation2d.fromDegrees(deg)); }

    private static ChassisSpeeds vel(double vx, double vy) { return new ChassisSpeeds(vx, vy, 0); }

    private static double angleBetween(Translation2d a, Translation2d b) {
        return Math.abs(MathUtil.angleModulus(Math.atan2(b.getY(), b.getX()) - Math.atan2(a.getY(), a.getX())));
    }

    private static double distanceToPath(Path path, Translation2d p) {
        double best = Double.POSITIVE_INFINITY;
        for (Segment seg : path.segments()) {
            double t = MathUtil.clamp(seg.project(p), 0, seg.length());
            best = Math.min(best, seg.pointAt(t).getDistance(p));
        }
        return best;
    }

    /** Point robot: velocity = command (lag 0) or a first-order lag of the command; records the run. */
    private static final class Sim {
        double maxDistance, maxJumpDeg, finishTime = Double.NaN, lastProgress, finalCmdSpeed = -1;
        boolean progressMonotonic = true;
        final List<Translation2d> positions = new ArrayList<>();
        final List<Double> cmdSpeeds = new ArrayList<>();
        final List<Translation2d> cmds = new ArrayList<>();
        final List<Double> arcSpeeds = new ArrayList<>();

        Sim(Path path, double lag, double maxTime) { this(path, lag, 0, 0, path.start(), maxTime); }

        /** delayLoops: command latency in loops; deadband: |command| below it doesn't move the robot. */
        Sim(Path path, double lag, int delayLoops, double deadband, Translation2d startPos, double maxTime) {
            PathFollower follower = new PathFollower(path, pose(startPos.getX(), startPos.getY(), 0));
            Translation2d pos = startPos, v = Translation2d.kZero, prevCmd = null;
            java.util.ArrayDeque<Translation2d> pipe = new java.util.ArrayDeque<>();
            for (int j = 0; j < delayLoops; j++) pipe.add(Translation2d.kZero);
            double theta = 0;
            for (int i = 0; i * DT < maxTime && !follower.isFinished(); i++) {
                ChassisSpeeds c = follower.calculate(new Pose2d(pos, new Rotation2d(theta)), vel(v.getX(), v.getY()));
                Translation2d cmd = new Translation2d(c.vxMetersPerSecond, c.vyMetersPerSecond);
                positions.add(pos);
                cmdSpeeds.add(cmd.getNorm());
                cmds.add(cmd);
                if (follower.progress() < lastProgress - 1e-12) progressMonotonic = false;
                lastProgress = follower.progress();
                maxDistance = Math.max(maxDistance, distanceToPath(path, pos));
                if (prevCmd != null && prevCmd.getNorm() > 0.2 && cmd.getNorm() > 0.2)
                    maxJumpDeg = Math.max(maxJumpDeg, Math.toDegrees(angleBetween(prevCmd, cmd)));
                prevCmd = cmd;
                if (path.segments().get(follower.segmentIndex()) instanceof Arc arc) { // speed along the path
                    Translation2d tan = arc.tangentAt(MathUtil.clamp(arc.project(pos), 0, arc.length()));
                    arcSpeeds.add(cmd.getX() * tan.getX() + cmd.getY() * tan.getY());
                }
                pipe.add(cmd);
                Translation2d applied = pipe.poll();
                if (applied.getNorm() < deadband) applied = Translation2d.kZero;
                v = lag == 0 ? applied : v.plus(applied.minus(v).times(DT / lag));
                pos = pos.plus(v.times(DT));
                theta += c.omegaRadiansPerSecond * DT;
                if (follower.isFinished()) finishTime = (i + 1) * DT;
            }
            ChassisSpeeds after = follower.calculate(new Pose2d(pos, new Rotation2d(theta)), vel(0, 0));
            finalCmdSpeed = Math.hypot(after.vxMetersPerSecond, after.vyMetersPerSecond);
        }
    }

    @Test
    void finishesWhenItStopsShortOfTheEndWhileStillTurning() {
        // The robot held within POSITION_TOLERANCE / 2 of the end with the heading of the path there (3.5 deg from
        // the final heading, which changes 63.5 deg over the last 0.25 m) and never finished.
        Path path = new Path(new Point(4.845, 0.913).stop(0.26),
                new Point(1.655, 1.956).withAngle(Rotation2d.fromDegrees(74)).withRadius(0.529),
                new Point(4.681, 2.88).withAngle(Rotation2d.fromDegrees(-113.5)),
                new Point(4.433, 2.907).withAngle(Rotation2d.fromDegrees(-177)));
        Sim sim = new Sim(path, 0, 30);
        assertTrue(sim.finishTime < 10, "finish time " + sim.finishTime);
    }

    @Test
    void legDirectionEqualsArcTangentAtEntryAndExit() {
        for (Path path : new Path[] {left(), right(), sCurve()}) {
            List<Segment> segs = path.segments();
            int arcs = 0;
            for (int k = 0; k < segs.size(); k++) {
                if (!(segs.get(k) instanceof Arc arc)) continue;
                arcs++;
                if (k > 0) {
                    Segment prev = segs.get(k - 1);
                    double deg = Math.toDegrees(angleBetween(prev.tangentAt(prev.length()), arc.tangentAt(0)));
                    assertTrue(deg < 0.01, "entry tangent, segment " + k + ": " + deg);
                }
                if (k < segs.size() - 1) {
                    double deg = Math.toDegrees(angleBetween(segs.get(k + 1).tangentAt(0), arc.tangentAt(arc.length())));
                    assertTrue(deg < 0.01, "exit tangent, segment " + k + ": " + deg);
                }
            }
            assertTrue(arcs >= 1);
        }
    }

    @Test
    void arcsPassThroughWaypointsWithTurnDirectionSigma() {
        Object[][] cases = {
            {left(), new Translation2d(2, 0), true},
            {right(), new Translation2d(2, 0), false},
            {sCurve(), new Translation2d(2, 0), true},
            {sCurve(), new Translation2d(2, 2), false},
        };
        for (Object[] c : cases) {
            Path path = (Path) c[0];
            Translation2d wp = (Translation2d) c[1];
            Arc best = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            for (Segment seg : path.segments()) {
                if (!(seg instanceof Arc arc)) continue;
                double d = arc.pointAt(MathUtil.clamp(arc.project(wp), 0, arc.length())).getDistance(wp);
                if (d < bestDistance) {
                    bestDistance = d;
                    best = arc;
                }
            }
            assertNotNull(best);
            assertEquals(0, bestDistance, 1e-6, "arc through " + wp);
            assertEquals((boolean) c[2], best.isLeftTurn(), "turn direction at " + wp);
        }
    }

    @Test
    void switchesFromLegToArcAtEntryNotBefore() {
        Path path = left();
        PathFollower f = new PathFollower(path, pose(0, 0, 0));
        assertInstanceOf(Line.class, path.segments().get(0));
        assertInstanceOf(Arc.class, path.segments().get(1));
        double entryX = path.segmentStart(1); // first leg lies on y = 0 from x = 0
        f.calculate(pose(entryX - 0.05, 0, 0), vel(1, 0));
        assertEquals(0, f.segmentIndex(), "0.05 m before entry");
        f.calculate(pose(entryX - 0.01, 0, 0), vel(1, 0));
        assertEquals(0, f.segmentIndex(), "0.01 m before entry");
        f.calculate(pose(entryX + 0.01, 0, 0), vel(1, 0));
        assertEquals(1, f.segmentIndex(), "just past entry");
    }

    @Test
    void simulatedRobotStaysOnLeftAndRightCornerPaths() {
        double worstDistance = 0, worstJump = 0;
        for (Path path : new Path[] {left(), right()})
            for (double lag : new double[] {0, 0.1}) {
                Sim sim = new Sim(path, lag, 10);
                String msg = "lag " + lag + " dist " + sim.maxDistance + " jump " + sim.maxJumpDeg;
                assertTrue(sim.maxDistance <= 0.05, msg);
                assertTrue(sim.maxJumpDeg <= 15, msg);
                assertTrue(sim.finishTime <= 10, "finish " + msg); // NaN fails
                worstDistance = Math.max(worstDistance, sim.maxDistance);
                worstJump = Math.max(worstJump, sim.maxJumpDeg);
            }
        System.out.printf("MEASURED max distance %.4f m, max heading jump %.2f deg%n", worstDistance, worstJump);
    }

    @Test
    void progressNeverDecreasesAndEndsAtPathLength() {
        for (Path path : new Path[] {left(), right()})
            for (double lag : new double[] {0, 0.1}) {
                Sim sim = new Sim(path, lag, 10);
                assertTrue(sim.progressMonotonic, "monotonic, lag " + lag);
                assertEquals(path.length(), sim.lastProgress, POSITION_TOLERANCE, "end progress, lag " + lag);
            }
    }

    @Test
    void speedBrakesToZeroAtStopDwellsThenContinuesAndEndsAtZero() {
        Path path = new Path(new Point(0, 0), new Point(2, 0).stop(0.5), new Point(4, 0));
        Sim sim = new Sim(path, 0, 10);
        double maxXBeforeStop = 0, minSpeedNearStop = Double.POSITIVE_INFINITY;
        int firstZero = -1, zeroLoops = 0;
        for (int i = 0; i < sim.positions.size(); i++) {
            double x = sim.positions.get(i).getX(), v = sim.cmdSpeeds.get(i);
            if (firstZero < 0 && x > 1.5 && v == 0) firstZero = i;
            if (firstZero < 0) {
                maxXBeforeStop = Math.max(maxXBeforeStop, x);
                if (x > 1.8) minSpeedNearStop = Math.min(minSpeedNearStop, v);
            }
            if (x > 1.5 && x < 2.5 && v == 0) zeroLoops++;
        }
        assertTrue(firstZero > 0, "never commanded a stop");
        assertTrue(maxXBeforeStop <= 2.05, "overshoot, x " + maxXBeforeStop);
        // braked from cruise down to the MIN_SPEED capture floor before the zero command at the stop
        assertTrue(minSpeedNearStop <= MIN_SPEED + 1e-9, "speed near stop " + minSpeedNearStop);
        assertTrue(zeroLoops * DT >= 0.5, "dwell " + zeroLoops * DT);
        assertTrue(sim.positions.get(sim.positions.size() - 1).getX() > 3.9, "continues past the stop");
        assertFalse(Double.isNaN(sim.finishTime), "finished");
        assertEquals(0, sim.finalCmdSpeed, 0, "speed after finish");
    }

    @Test
    void headingFinishUsesAbsoluteError() {
        Path path = new Path(new Point(0, 0), new Point(2, 0).withAngle(Rotation2d.fromDegrees(10)));
        double[][] cases = {{15, 0}, {5, 0}, {10, 1}, {11, 1}, {9, 1}, {13, 0}};
        for (double[] c : cases) {
            PathFollower f = new PathFollower(path, pose(0, 0, 0));
            f.calculate(pose(2, 0, c[0]), vel(0, 0));
            assertEquals(c[1] == 1, f.isFinished(), "heading " + c[0] + " deg");
        }
    }

    @Test
    void flippedPathRotateAndMirror() {
        double L = Point.Field.layout().getFieldLength(), W = Point.Field.layout().getFieldWidth();
        for (boolean mirror : new boolean[] {false, true}) {
            Path flipped = new Path(new Point(1, 1), new Point(3, 2).withAngle(Rotation2d.fromDegrees(30))).flipped(mirror);
            Translation2d start = mirror ? new Translation2d(L - 1, 1) : new Translation2d(L - 1, W - 1);
            Translation2d end = mirror ? new Translation2d(L - 3, 2) : new Translation2d(L - 3, W - 2);
            assertEquals(0, flipped.start().getDistance(start), 1e-9, "start, mirror " + mirror);
            assertEquals(0, flipped.end().getDistance(end), 1e-9, "end, mirror " + mirror);
            Translation2d expected = end.minus(start);
            assertEquals(0, flipped.segments().get(0).tangentAt(0).getDistance(expected.div(expected.getNorm())), 1e-9);
            double angle = flipped.anchors().get(flipped.anchors().size() - 1).angle().getDegrees();
            assertEquals(mirror ? 150 : 210, MathUtil.inputModulus(angle, 0, 360), 1e-9, "angle, mirror " + mirror);

            Arc arc = (Arc) left().flipped(mirror).segments().stream().filter(x -> x instanceof Arc).findFirst().orElseThrow();
            assertEquals(!mirror, arc.isLeftTurn(), "mirror turns a left turn into a right turn");
        }
        assertEquals(0, left().flipped().start().getDistance(left().flipped(MIRRORED_FIELD).start()), 1e-12);
    }

    @Test
    void closeSameDirectionCornersAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new Path(new Point(0, 0), new Point(2, 0), new Point(2.05, 0.05), new Point(2.05, 2)));
    }

    @Test
    void stopsAndEndFinishWithDriveDeadband() {
        Path path = new Path(new Point(0, 0), new Point(2, 0).stop(), new Point(2, 2));
        for (double lag : new double[] {0, 0.1}) {
            Sim sim = new Sim(path, lag, 0, 0.1, path.start(), 15);
            assertFalse(Double.isNaN(sim.finishTime), "finished, lag " + lag);
            assertTrue(sim.positions.get(sim.positions.size() - 1).getDistance(path.end()) < POSITION_TOLERANCE);
        }
    }

    @Test
    void stopOvershootUnderHeavyLagSettles() {
        Path path = new Path(new Point(0, 0), new Point(3, 0).stop(), new Point(3, 3));
        Sim sim = new Sim(path, 0.1, 10, 0, path.start(), 20); // 200 ms latency + 0.1 s lag
        assertFalse(Double.isNaN(sim.finishTime), "finished");
    }

    @Test
    void startPastTheEndDrivesBack() {
        Path path = new Path(new Point(0, 0), new Point(2, 0));
        Sim sim = new Sim(path, 0.1, 0, 0, new Translation2d(2.1, 0), 5);
        assertFalse(Double.isNaN(sim.finishTime), "finished");
        assertTrue(sim.positions.get(sim.positions.size() - 1).getDistance(path.end()) < POSITION_TOLERANCE);
    }

    @Test
    void arcSpeedRespectsLateralAccelerationLimit() {
        Path path = new Path(new Point(0, 0), new Point(6, 0), new Point(6, 6));
        Sim sim = new Sim(path, 0, 15);
        double cap = Math.sqrt(MAX_ACCELERATION * RADIUS);
        assertTrue(cap < MAX_VELOCITY, "test needs a cap below MAX_VELOCITY");
        assertFalse(sim.arcSpeeds.isEmpty(), "never on the arc");
        for (double v : sim.arcSpeeds) assertTrue(v <= cap + 1e-6, "arc speed " + v + " > cap " + cap);
    }

    @Test
    void mergesOntoLegAlongAGentleCurve() {
        // start 0.3 m left of a long straight leg: the robot should curve onto it, not drive straight at it
        Path path = new Path(new Point(0, 0), new Point(8, 0));
        for (double lag : new double[] {0, 0.1}) {
            Sim sim = new Sim(path, lag, 0, 0, new Translation2d(0.5, 0.3), 15);
            double maxAngle = 0, minY = Double.POSITIVE_INFINITY;
            for (int i = 0; i < sim.cmds.size(); i++) {
                Translation2d c = sim.cmds.get(i);
                if (sim.positions.get(i).getX() < 7 && c.getNorm() > 0.05)
                    maxAngle = Math.max(maxAngle, Math.toDegrees(Math.abs(Math.atan2(c.getY(), c.getX()))));
                minY = Math.min(minY, sim.positions.get(i).getY());
            }
            System.out.printf("MERGE lag=%.1f maxAngle=%.1f deg minY=%.4f m%n", lag, maxAngle, minY);
            double limit = Math.toDegrees(Math.atan(0.3 / MERGE_DISTANCE)) + 4; // the merge angle at the start, plus margin
            assertTrue(maxAngle <= limit, "approach angle " + maxAngle + " deg > " + limit + ", lag " + lag);
            assertTrue(minY > -0.02, "crossed the leg by " + (-minY) + " m, lag " + lag);
            assertFalse(Double.isNaN(sim.finishTime), "finished, lag " + lag);
        }
    }
}
