package frc.robot.Kinematics;

import static frc.robot.Kinematics.KinematicsConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** The chassis methods: toSwerveModuleStates (omega clockwise-positive), cosine scaling, accelerations. */
class ChassisKinematicsTest {
    private static final double EPSILON = 1e-9;

    private final DemaciaKinematics kinematics = new DemaciaKinematics(DemaciaKinematicsTest.MODULE_LOCATIONS);

    private static SwerveModuleState[] measured(double degrees) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(degrees));
        }
        return states;
    }

    private static void assertSameVelocity(SwerveModuleState expected, SwerveModuleState actual, String message) {
        assertEquals(expected.speedMetersPerSecond * expected.angle.getCos(),
                actual.speedMetersPerSecond * actual.angle.getCos(), EPSILON, message + " vx");
        assertEquals(expected.speedMetersPerSecond * expected.angle.getSin(),
                actual.speedMetersPerSecond * actual.angle.getSin(), EPSILON, message + " vy");
    }

    @Test
    void omegaIsClockwisePositive() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 2));
        SwerveModuleState[] wpilib = kinematics.toWpilibModuleStates(new ChassisSpeeds(0, 0, -2));
        for (int i = 0; i < 4; i++) {
            assertSameVelocity(wpilib[i], states[i], "module " + i);
        }
    }

    @Test
    void straightDrivingIsPlainKinematics() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(1.5, -1, 0));
        SwerveModuleState[] wpilib = kinematics.toWpilibModuleStates(new ChassisSpeeds(1.5, -1, 0));
        for (int i = 0; i < 4; i++) {
            assertSameVelocity(wpilib[i], states[i], "module " + i);
        }
    }

    @Test
    void driveAndSpinIsDiscretizedOverTheLoopAndTheLatency() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(2, 0, 3));
        SwerveModuleState[] expected = kinematics.toWpilibModuleStates(
                ChassisSpeeds.discretize(new ChassisSpeeds(2, 0, -3), CYCLE_DT + 2 * LATENCY));
        for (int i = 0; i < 4; i++) {
            assertSameVelocity(expected[i], states[i], "module " + i);
        }
    }

    @Test
    void standingKeepsTheAngles() {
        SwerveModuleState[] driving = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 1, 0));
        SwerveModuleState[] standing = kinematics.toSwerveModuleStates(new ChassisSpeeds());
        for (int i = 0; i < 4; i++) {
            assertEquals(0, standing[i].speedMetersPerSecond, EPSILON);
            assertEquals(driving[i].angle.getRadians(), standing[i].angle.getRadians(), EPSILON);
        }
    }

    @Test
    void cosineScalingAgainstTheMeasuredAngle() {
        ChassisSpeeds forward = new ChassisSpeeds(2, 0, 0);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(forward, measured(0));
        assertEquals(2, states[0].speedMetersPerSecond, EPSILON);
        states = kinematics.toSwerveModuleStates(forward, measured(60));
        assertEquals(1, states[0].speedMetersPerSecond, EPSILON);
        assertEquals(0, states[0].angle.getDegrees(), EPSILON);
        states = kinematics.toSwerveModuleStates(forward, measured(90));
        assertEquals(0, states[0].speedMetersPerSecond, EPSILON);
    }

    @Test
    void reversesWhenTheMeasuredAngleIsMoreThan90DegreesAway() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(2, 0, 0), measured(120));
        // turn to 180 and drive backwards, at cos(60) = half speed
        assertEquals(180, Math.abs(states[0].angle.getDegrees()), 1e-6);
        assertEquals(-1, states[0].speedMetersPerSecond, EPSILON);
    }

    @Test
    void driveAccelerationsFromTheCommands() {
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0.2, 0, 0), measured(0));
        assertEquals(0.2 / CYCLE_DT, kinematics.getDriveAccelerations()[0], EPSILON);
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0.2, 0, 0), measured(0));
        assertEquals(0, kinematics.getDriveAccelerations()[0], EPSILON);
        kinematics.toSwerveModuleStates(new ChassisSpeeds(3, 0, 0), measured(0));
        assertEquals(MAX_DRIVE_ACCEL, kinematics.getDriveAccelerations()[0], EPSILON); // clamped
        // the wheel reverses (module turned around): 3 m/s forward is -3 along the new angle
        kinematics.toSwerveModuleStates(new ChassisSpeeds(3, 0, 0), measured(180));
        assertEquals(-3, kinematics.toSwerveModuleStates(new ChassisSpeeds(3, 0, 0), measured(180))[0]
                .speedMetersPerSecond, EPSILON);
        assertEquals(0, kinematics.getDriveAccelerations()[0], EPSILON);
    }

    @Test
    void steerVelocitiesFromTheCommands() {
        kinematics.toSwerveModuleStates(new ChassisSpeeds(1, 0, 0), measured(0));
        // 0.1 rad in one loop = 5 rad/s
        kinematics.toSwerveModuleStates(new ChassisSpeeds(Math.cos(0.1), Math.sin(0.1), 0), measured(0));
        assertEquals(0.1 / CYCLE_DT, kinematics.getSteerVelocities()[0], 1e-6);
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 1, 0), measured(0));
        assertEquals(MAX_STEER_RATE, kinematics.getSteerVelocities()[0], EPSILON); // clamped
        kinematics.toSwerveModuleStates(new ChassisSpeeds(), measured(90));
        assertEquals(0, kinematics.getSteerVelocities()[0], EPSILON); // standing holds the angle
    }
}
