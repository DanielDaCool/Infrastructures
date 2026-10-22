package frc.robot.Kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class ModuleCommandTest {
    private static final double DT = 0.02;
    private static final double EPSILON = 1e-9;
    private static final int N = 4;
    private static final ModuleCommandConfig NO_LATENCY = new ModuleCommandConfig(0, 0, 12, 20, false);
    private static final ModuleCommandConfig WITH_STEER_FF = new ModuleCommandConfig(0, 0, 12, 20, true);

    private final DemaciaKinematics kinematics = new DemaciaKinematics(DemaciaKinematicsTest.MODULE_LOCATIONS);

    private static Rotation2d[] all(Rotation2d angle) {
        Rotation2d[] angles = new Rotation2d[N];
        java.util.Arrays.fill(angles, angle);
        return angles;
    }

    private static ModuleCommand[] previous(double speed, Rotation2d... angles) {
        ModuleCommand[] commands = new ModuleCommand[N];
        for (int i = 0; i < N; i++) {
            commands[i] = new ModuleCommand(angles[i], speed, 0, 0);
        }
        return commands;
    }

    /** Robot speeds that make every module drive at the given angle at 1 m/s (no rotation). */
    private static ChassisSpeeds toward(Rotation2d angle) {
        return new ChassisSpeeds(angle.getCos(), angle.getSin(), 0);
    }

    private ModuleCommand[] commands(ChassisSpeeds speeds, Rotation2d[] measured, ModuleCommand[] previous,
            boolean[] reversed, ModuleCommandConfig config) {
        return kinematics.toModuleCommands(speeds, measured, new double[N], previous, reversed, config, DT);
    }

    @Test
    void zeroLatencyWithMeasuredAtTheTargetsMatchesTheExistingCalculate() {
        AccelLimitedSwerve plain = new AccelLimitedSwerve(kinematics);
        AccelLimitedSwerve aware = new AccelLimitedSwerve(kinematics, NO_LATENCY);
        ChassisSpeeds wanted = new ChassisSpeeds(2, 1, 0); // no spin: the modules stay within 90 degrees of 0
        for (int loop = 0; loop < 40; loop++) {
            SwerveModuleState[] expected = plain.calculate(wanted, Rotation2d.kZero, DT);
            Rotation2d[] measured = new Rotation2d[N];
            for (int i = 0; i < N; i++) {
                measured[i] = expected[i].angle;
            }
            ModuleCommand[] actual = aware.calculate(wanted, Rotation2d.kZero, 0, measured, new double[N], DT);
            for (int i = 0; i < N; i++) {
                assertEquals(expected[i].speedMetersPerSecond, actual[i].speedMetersPerSecond(), EPSILON);
                assertEquals(expected[i].angle.getRadians(), actual[i].angle().getRadians(), EPSILON);
            }
        }
    }

    @Test
    void latencyCompensationTurnsTheCommandByMinusOmegaTimesLatency() {
        double omega = 2;
        ModuleCommandConfig config = new ModuleCommandConfig(0.035, 0, 12, 20, false);
        AccelLimitedSwerve drive = new AccelLimitedSwerve(kinematics, config);
        ChassisSpeeds wanted = new ChassisSpeeds(3, 0, 0);
        Rotation2d[] measured = all(Rotation2d.kZero);
        ModuleCommand[] commands = null;
        for (int loop = 0; loop < 100; loop++) {
            commands = drive.calculate(wanted, Rotation2d.kZero, omega, measured, new double[N], DT);
            for (int i = 0; i < N; i++) {
                measured[i] = commands[i].angle();
            }
        }
        double turn = omega * config.latencySeconds();
        ChassisSpeeds robotRel = drive.getSentSpeedsRobotRel();
        assertEquals(3 * Math.cos(turn), robotRel.vxMetersPerSecond, 1e-6);
        assertEquals(-3 * Math.sin(turn), robotRel.vyMetersPerSecond, 1e-6);
        for (ModuleCommand command : commands) {
            assertEquals(-turn, command.angle().getRadians(), 1e-6);
            assertEquals(3, command.speedMetersPerSecond(), 1e-6);
        }
    }

    @Test
    void cosineScalingSlowsAModuleThatIsOffTarget() {
        double[] speeds = new double[3];
        double[] offsets = { 0, 60, 90 };
        for (int k = 0; k < offsets.length; k++) {
            AccelLimitedSwerve drive = new AccelLimitedSwerve(kinematics, NO_LATENCY);
            Rotation2d measuredAngle = Rotation2d.fromDegrees(offsets[k]);
            SwerveModuleState[] measuredStates = new SwerveModuleState[N];
            java.util.Arrays.fill(measuredStates, new SwerveModuleState(0, Rotation2d.kZero));
            drive.resetTo(new ChassisSpeeds(), measuredStates);
            speeds[k] = drive.calculate(new ChassisSpeeds(3, 0, 0), Rotation2d.kZero, 0, all(measuredAngle),
                    new double[N], DT)[0].speedMetersPerSecond();
        }
        assertTrue(speeds[0] > 0.1, "planned speed " + speeds[0]);
        assertEquals(speeds[0] / 2, speeds[1], EPSILON);
        assertEquals(0, speeds[2], EPSILON);
    }

    @Test
    void cosineScalingAdvancesTheMeasuredAngleByItsLatency() {
        ModuleCommandConfig config = new ModuleCommandConfig(0, 0.05, 12, 20, false);
        // measured 40 degrees behind, turning 10 rad/s: 0.5 rad = 28.6 degrees ahead at 11.4 degrees
        double[] steerVelocities = new double[N];
        java.util.Arrays.fill(steerVelocities, 10);
        ModuleCommand[] commands = kinematics.toModuleCommands(toward(Rotation2d.kZero), all(Rotation2d.fromDegrees(-40)),
                steerVelocities, previous(0, all(Rotation2d.kZero)), new boolean[N], config, DT);
        double error = Math.toRadians(-40) + 0.5;
        assertEquals(Math.cos(error), commands[0].speedMetersPerSecond(), EPSILON);
    }

    @Test
    void reversesWhenTheTargetIsMoreThan90DegreesFromThePreviousCommand() {
        for (double degrees : new double[] { 95, 85 }) {
            Rotation2d target = Rotation2d.fromDegrees(degrees);
            boolean expectReverse = degrees > 90;
            Rotation2d commanded = expectReverse ? target.plus(Rotation2d.fromDegrees(180)) : target;
            boolean[] reversed = new boolean[N];
            ModuleCommand[] commands = commands(toward(target), all(commanded), previous(0, all(Rotation2d.kZero)),
                    reversed, NO_LATENCY);
            for (int i = 0; i < N; i++) {
                assertEquals(expectReverse ? -1 : 1, commands[i].speedMetersPerSecond(), EPSILON);
                assertEquals(0, commands[i].angle().minus(commanded).getRadians(), EPSILON);
                assertTrue(expectReverse == reversed[i]);
            }
        }
    }

    @Test
    void aModuleFarFromTheLimitDecidesAlone() {
        // target 0; previous angles 120, 89, 89, 89 away: only the first reverses
        Rotation2d[] prev = { Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(89), Rotation2d.fromDegrees(89),
                Rotation2d.fromDegrees(89) };
        boolean[] reversed = new boolean[N];
        commands(toward(Rotation2d.kZero), all(Rotation2d.kZero), previous(0, prev), reversed, NO_LATENCY);
        assertTrue(reversed[0]);
        assertTrue(!reversed[1] && !reversed[2] && !reversed[3]);
    }

    @Test
    void modulesNearThe90DegreeLimitTakeTheMajorityDecision() {
        // target 0; the previous angles are 89, 91, 91, 91 degrees away: three of four would reverse
        Rotation2d[] prev = { Rotation2d.fromDegrees(-89), Rotation2d.fromDegrees(-91), Rotation2d.fromDegrees(-91),
                Rotation2d.fromDegrees(-91) };
        boolean[] reversed = new boolean[N];
        ModuleCommand[] commands = commands(toward(Rotation2d.kZero), all(Rotation2d.fromDegrees(180)),
                previous(0, prev), reversed, NO_LATENCY);
        for (int i = 0; i < N; i++) {
            assertTrue(reversed[i], "module " + i);
            assertEquals(-1, commands[i].speedMetersPerSecond(), EPSILON);
            assertEquals(Math.PI, Math.abs(commands[i].angle().getRadians()), EPSILON);
        }
        // and the other way: 89, 89, 89, 91 away: nobody reverses
        prev = new Rotation2d[] { Rotation2d.fromDegrees(-89), Rotation2d.fromDegrees(-89),
                Rotation2d.fromDegrees(-89), Rotation2d.fromDegrees(-91) };
        reversed = new boolean[N];
        commands = commands(toward(Rotation2d.kZero), all(Rotation2d.kZero), previous(0, prev), reversed,
                NO_LATENCY);
        for (int i = 0; i < N; i++) {
            assertTrue(!reversed[i], "module " + i);
            assertEquals(1, commands[i].speedMetersPerSecond(), EPSILON);
        }
    }

    @Test
    void aTieKeepsThePreviousReversedFlags() {
        // 89, 89, 91, 91 away: two and two
        Rotation2d[] prev = { Rotation2d.fromDegrees(-89), Rotation2d.fromDegrees(-89), Rotation2d.fromDegrees(-91),
                Rotation2d.fromDegrees(-91) };
        boolean[] reversed = { true, false, false, true };
        boolean[] before = reversed.clone();
        Rotation2d[] measured = { Rotation2d.fromDegrees(180), Rotation2d.kZero, Rotation2d.kZero,
                Rotation2d.fromDegrees(180) };
        ModuleCommand[] commands = commands(toward(Rotation2d.kZero), measured, previous(0, prev), reversed,
                NO_LATENCY);
        for (int i = 0; i < N; i++) {
            assertTrue(before[i] == reversed[i], "module " + i);
            assertEquals(before[i] ? -1 : 1, commands[i].speedMetersPerSecond(), EPSILON);
        }
    }

    @Test
    void driveAccelerationFeedforwardIsThePlannedSpeedChangeAndClamped() {
        Rotation2d[] zero = all(Rotation2d.kZero);
        ChassisSpeeds slow = new ChassisSpeeds(0.2, 0, 0);
        ModuleCommand[] commands = commands(slow, zero, previous(0, zero), new boolean[N], NO_LATENCY);
        assertEquals(0.2 / DT, commands[0].accelerationMetersPerSecondSquared(), EPSILON);

        commands = commands(new ChassisSpeeds(1, 0, 0), zero, previous(0, zero), new boolean[N], NO_LATENCY);
        assertEquals(20, commands[0].accelerationMetersPerSecondSquared(), EPSILON);

        commands = commands(new ChassisSpeeds(0.5, 0, 0), zero, previous(1, zero), new boolean[N], NO_LATENCY);
        assertEquals(-20, commands[0].accelerationMetersPerSecondSquared(), EPSILON);

        commands = commands(new ChassisSpeeds(0.9, 0, 0), zero, previous(1, zero), new boolean[N], NO_LATENCY);
        assertEquals(-0.1 / DT, commands[0].accelerationMetersPerSecondSquared(), EPSILON);
    }

    @Test
    void steerVelocityFeedforwardIsOffByDefaultAndClamped() {
        Rotation2d[] zero = all(Rotation2d.kZero);
        Rotation2d small = Rotation2d.fromRadians(0.1);
        ModuleCommand[] off = commands(toward(small), all(small), previous(0, zero), new boolean[N], NO_LATENCY);
        assertEquals(0, off[0].steerVelocityRadiansPerSecond(), EPSILON);

        ModuleCommand[] on = commands(toward(small), all(small), previous(0, zero), new boolean[N], WITH_STEER_FF);
        assertEquals(0.1 / DT, on[0].steerVelocityRadiansPerSecond(), EPSILON);

        Rotation2d big = Rotation2d.fromRadians(0.5);
        on = commands(toward(big), all(big), previous(0, zero), new boolean[N], WITH_STEER_FF);
        assertEquals(12, on[0].steerVelocityRadiansPerSecond(), EPSILON);

        Rotation2d negative = Rotation2d.fromRadians(-0.5);
        on = commands(toward(negative), all(negative), previous(0, zero), new boolean[N], WITH_STEER_FF);
        assertEquals(-12, on[0].steerVelocityRadiansPerSecond(), EPSILON);
    }

    @Test
    void nanMeasurementsDoNotScaleOrPoisonTheCommand() {
        Rotation2d[] zero = all(Rotation2d.kZero);
        Rotation2d[] measured = zero.clone();
        measured[0] = new Rotation2d(Double.NaN);
        double[] steerVelocities = new double[N];
        steerVelocities[1] = Double.POSITIVE_INFINITY;
        steerVelocities[2] = Double.NaN;
        ModuleCommand[] commands = kinematics.toModuleCommands(toward(Rotation2d.kZero), measured, steerVelocities,
                previous(0, zero), new boolean[N], NO_LATENCY, DT);
        for (ModuleCommand command : commands) {
            assertEquals(1, command.speedMetersPerSecond(), EPSILON);
            assertEquals(0, command.angle().getRadians(), EPSILON);
            assertTrue(Double.isFinite(command.accelerationMetersPerSecondSquared()));
        }
    }

    @Test
    void nanGyroRateCountsAsZero() {
        ModuleCommandConfig config = new ModuleCommandConfig(0.035, 0, 12, 20, false);
        AccelLimitedSwerve nan = new AccelLimitedSwerve(kinematics, config);
        AccelLimitedSwerve zero = new AccelLimitedSwerve(kinematics, config);
        Rotation2d[] measured = all(Rotation2d.kZero);
        ModuleCommand[] a = nan.calculate(new ChassisSpeeds(1, 1, 0), Rotation2d.kZero, Double.NaN, measured,
                new double[N], DT);
        ModuleCommand[] b = zero.calculate(new ChassisSpeeds(1, 1, 0), Rotation2d.kZero, 0, measured, new double[N],
                DT);
        for (int i = 0; i < N; i++) {
            assertEquals(b[i].speedMetersPerSecond(), a[i].speedMetersPerSecond(), EPSILON);
            assertEquals(b[i].angle().getRadians(), a[i].angle().getRadians(), EPSILON);
        }
    }

    @Test
    void standingKeepsTheAnglesAndBrakesWithTheFeedforward() {
        AccelLimitedSwerve drive = new AccelLimitedSwerve(kinematics, NO_LATENCY);
        SwerveModuleState[] measuredStates = new SwerveModuleState[N];
        java.util.Arrays.fill(measuredStates, new SwerveModuleState(1, Rotation2d.fromDegrees(90)));
        drive.resetTo(new ChassisSpeeds(), measuredStates);
        ModuleCommand[] commands = drive.calculate(new ChassisSpeeds(), Rotation2d.kZero, 0, all(Rotation2d.kZero),
                new double[N], DT);
        for (ModuleCommand command : commands) {
            assertEquals(0, command.speedMetersPerSecond(), EPSILON);
            assertEquals(Math.PI / 2, command.angle().getRadians(), EPSILON);
            assertEquals(-20, command.accelerationMetersPerSecondSquared(), EPSILON); // -1 / 0.02 clamped
            assertEquals(0, command.steerVelocityRadiansPerSecond(), EPSILON);
        }
    }

    @Test
    void toStateGivesTheAngleAndSpeed() {
        SwerveModuleState state = new ModuleCommand(Rotation2d.fromDegrees(30), 2, 3, 4).toState();
        assertEquals(2, state.speedMetersPerSecond, EPSILON);
        assertEquals(Math.toRadians(30), state.angle.getRadians(), EPSILON);
    }

    @Test
    void badInputIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(-0.01, 0, 12, 20, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(Double.NaN, 0, 12, 20, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(0, Double.POSITIVE_INFINITY, 12, 20, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(0, -1, 12, 20, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(0, 0, 0, 20, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(0, 0, Double.NaN, 20, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(0, 0, 12, 0, false));
        assertThrows(IllegalArgumentException.class, () -> new ModuleCommandConfig(0, 0, 12, Double.POSITIVE_INFINITY, false));
        ModuleCommandConfig defaults = ModuleCommandConfig.defaults();
        assertEquals(0.035, defaults.latencySeconds(), EPSILON);
        assertEquals(12, defaults.maxSteerRateRadiansPerSecond(), EPSILON);

        AccelLimitedSwerve drive = new AccelLimitedSwerve(kinematics);
        assertThrows(IllegalArgumentException.class, () -> drive.calculate(new ChassisSpeeds(1, 0, 0),
                Rotation2d.kZero, 0, new Rotation2d[3], new double[N], DT));
        assertThrows(IllegalArgumentException.class, () -> drive.calculate(new ChassisSpeeds(1, 0, 0),
                Rotation2d.kZero, 0, all(Rotation2d.kZero), new double[5], DT));
        assertThrows(IllegalArgumentException.class, () -> kinematics.toModuleCommands(new ChassisSpeeds(1, 0, 0),
                all(Rotation2d.kZero), new double[N], previous(0, all(Rotation2d.kZero)), new boolean[2],
                NO_LATENCY, DT));
    }
}
