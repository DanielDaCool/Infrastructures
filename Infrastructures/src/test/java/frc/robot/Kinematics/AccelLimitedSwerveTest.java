package frc.robot.Kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class AccelLimitedSwerveTest {
    private static final double DT = 0.02;
    private static final double EPSILON = 1e-9;
    // the most the speeds can change in one loop (the skid limit)
    private static final double MAX_STEP = SwerveAccelerationLimiter.MAX_SKID_ACCEL * DT;

    private final AccelLimitedSwerve drive =
            new AccelLimitedSwerve(new DemaciaKinematics(DemaciaKinematicsTest.MODULE_LOCATIONS));

    /** Drives until the sent speeds reach the wanted ones, returns the number of loops. */
    private int driveUntil(ChassisSpeeds wantedFieldRel, Rotation2d heading, int maxLoops) {
        for (int loop = 1; loop <= maxLoops; loop++) {
            drive.calculate(wantedFieldRel, heading, DT);
            ChassisSpeeds sent = drive.getSentSpeedsFieldRel();
            if (Math.abs(sent.vxMetersPerSecond - wantedFieldRel.vxMetersPerSecond) < EPSILON
                    && Math.abs(sent.vyMetersPerSecond - wantedFieldRel.vyMetersPerSecond) < EPSILON) {
                return loop;
            }
        }
        return -1;
    }

    private static double distance(ChassisSpeeds a, ChassisSpeeds b) {
        return Math.hypot(a.vxMetersPerSecond - b.vxMetersPerSecond, a.vyMetersPerSecond - b.vyMetersPerSecond);
    }

    @Test
    void reachesTheWantedSpeedFromRest() {
        int loops = driveUntil(new ChassisSpeeds(0, 3, 0), Rotation2d.kZero, 100);
        assertTrue(loops >= 20 && loops <= 30, "reached 3 m/s after " + loops + " loops");
        assertEquals(3, drive.getSentSpeedsFieldRel().vyMetersPerSecond, EPSILON);
        assertEquals(0, drive.getSentSpeedsFieldRel().vxMetersPerSecond, EPSILON);
    }

    @Test
    void convergesWhenTheRobotIsTurned() {
        Rotation2d heading = Rotation2d.fromDegrees(30);
        assertTrue(driveUntil(new ChassisSpeeds(3, 0, 0), heading, 100) > 0);
        ChassisSpeeds robotRel = drive.getSentSpeedsRobotRel();
        // field (3, 0) seen from a robot turned 30 degrees CCW
        assertEquals(3 * Math.cos(Math.toRadians(30)), robotRel.vxMetersPerSecond, EPSILON);
        assertEquals(-3 * Math.sin(Math.toRadians(30)), robotRel.vyMetersPerSecond, EPSILON);
    }

    @Test
    void convergesWhileSpinning() {
        ChassisSpeeds wanted = new ChassisSpeeds(3, 0, 3);
        Rotation2d heading = Rotation2d.kZero;
        for (int loop = 0; loop < 100; loop++) {
            drive.calculate(wanted, heading, DT);
            heading = heading.plus(Rotation2d.fromRadians(wanted.omegaRadiansPerSecond * DT));
        }
        assertEquals(0, distance(wanted, drive.getSentSpeedsFieldRel()), 1e-6);
    }

    @Test
    void yawResetDoesNotJump() {
        driveUntil(new ChassisSpeeds(3, 0, 0), Rotation2d.kZero, 100);
        ChassisSpeeds before = drive.getSentSpeedsRobotRel();

        // the gyro is set to 90 degrees, the driver keeps asking for field (3, 0)
        Rotation2d newYaw = Rotation2d.fromDegrees(90);
        drive.onYawReset(Rotation2d.kZero, newYaw);
        assertEquals(0, distance(before, drive.getSentSpeedsRobotRel()), EPSILON);

        for (int loop = 0; loop < 50; loop++) {
            drive.calculate(new ChassisSpeeds(3, 0, 0), newYaw, DT);
            ChassisSpeeds after = drive.getSentSpeedsRobotRel();
            assertTrue(distance(before, after) <= MAX_STEP + EPSILON,
                    "loop " + loop + " stepped " + distance(before, after));
            before = after;
        }
    }

    @Test
    void stoppingKeepsTheModuleAngles() {
        driveUntil(new ChassisSpeeds(1, 1, 0), Rotation2d.kZero, 100);
        SwerveModuleState[] states = drive.calculate(new ChassisSpeeds(1, 1, 0), Rotation2d.kZero, DT);
        double angle = states[0].angle.getRadians();
        assertEquals(Math.PI / 4, angle, EPSILON);

        // brake until the robot stands, the modules must not turn back to 0
        for (int loop = 0; loop < 100; loop++) {
            states = drive.calculate(new ChassisSpeeds(), Rotation2d.kZero, DT);
        }
        for (SwerveModuleState state : states) {
            assertEquals(0, state.speedMetersPerSecond, EPSILON);
            assertEquals(angle, state.angle.getRadians(), EPSILON);
        }
        assertEquals(0, distance(new ChassisSpeeds(), drive.getSentSpeedsFieldRel()), EPSILON);
    }

    @Test
    void resetStartsFromTheMeasuredSpeeds() {
        drive.resetTo(new ChassisSpeeds(2, 0, 0));
        drive.calculate(new ChassisSpeeds(2, 0, 0), Rotation2d.kZero, DT);
        assertEquals(2, drive.getSentSpeedsFieldRel().vxMetersPerSecond, EPSILON);
    }
}
