package frc.robot.Kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class DemaciaKinematicsTest {
    // Robot C
    static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(0.32, 0.27), new Translation2d(0.32, -0.27),
        new Translation2d(-0.32, 0.27), new Translation2d(-0.32, -0.27) };

    private static final double EPSILON = 1e-9;
    private static final ChassisSpeeds[] SPEEDS = {
        new ChassisSpeeds(0, 1, 0), new ChassisSpeeds(0, 0, 1),
        new ChassisSpeeds(2, 0, 2), new ChassisSpeeds(1.5, -1, 3) };

    private final DemaciaKinematics kinematics = new DemaciaKinematics(MODULE_LOCATIONS);

    /** Module velocity vector (vx, vy) of a state. */
    private static double[] vector(SwerveModuleState state) {
        return new double[] { state.speedMetersPerSecond * state.angle.getCos(),
                state.speedMetersPerSecond * state.angle.getSin() };
    }

    @Test
    void moduleStatesMatchWpilib() {
        for (ChassisSpeeds speeds : SPEEDS) {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
            for (int i = 0; i < 4; i++) {
                // WPILib: module at (x, y) moves at (vx - omega * y, vy + omega * x)
                double vx = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * MODULE_LOCATIONS[i].getY();
                double vy = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * MODULE_LOCATIONS[i].getX();
                double[] v = vector(states[i]);
                assertEquals(vx, v[0], EPSILON, speeds + " module " + i + " vx");
                assertEquals(vy, v[1], EPSILON, speeds + " module " + i + " vy");
            }
        }
    }

    @Test
    void forwardIsTheInverseOfInverse() {
        for (ChassisSpeeds speeds : SPEEDS) {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

            ChassisSpeeds withGyro = kinematics.toChassisSpeeds(states, speeds.omegaRadiansPerSecond);
            assertEquals(speeds.vxMetersPerSecond, withGyro.vxMetersPerSecond, EPSILON, speeds + " vx");
            assertEquals(speeds.vyMetersPerSecond, withGyro.vyMetersPerSecond, EPSILON, speeds + " vy");
            assertEquals(speeds.omegaRadiansPerSecond, withGyro.omegaRadiansPerSecond, EPSILON, speeds + " omega");

            ChassisSpeeds noGyro = kinematics.toChassisSpeeds(states);
            assertEquals(speeds.vxMetersPerSecond, noGyro.vxMetersPerSecond, EPSILON, speeds + " vx");
            assertEquals(speeds.vyMetersPerSecond, noGyro.vyMetersPerSecond, EPSILON, speeds + " vy");
            assertEquals(speeds.omegaRadiansPerSecond, noGyro.omegaRadiansPerSecond, EPSILON, speeds + " omega");
        }
    }

    @Test
    void forwardIsTheInverseForAnOffCenterLayout() {
        Translation2d[] offCenter = {
            new Translation2d(0.5, 0.3), new Translation2d(0.5, -0.2),
            new Translation2d(-0.1, 0.3), new Translation2d(-0.1, -0.2) };
        DemaciaKinematics offCenterKinematics = new DemaciaKinematics(offCenter);
        ChassisSpeeds speeds = new ChassisSpeeds(1.5, -1, 3);

        ChassisSpeeds result = offCenterKinematics.toChassisSpeeds(offCenterKinematics.toSwerveModuleStates(speeds));
        assertEquals(1.5, result.vxMetersPerSecond, EPSILON);
        assertEquals(-1, result.vyMetersPerSecond, EPSILON);
        assertEquals(3, result.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void desaturatesWhenOneModuleIsAtZero() {
        // spinning around the front left module: it stands, the back right is the fastest
        double omega = 20;
        Translation2d frontLeft = MODULE_LOCATIONS[0];
        ChassisSpeeds speeds = new ChassisSpeeds(omega * frontLeft.getY(), -omega * frontLeft.getX(), omega);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        assertEquals(0, states[0].speedMetersPerSecond, EPSILON);
        double max = 0;
        for (SwerveModuleState state : states) max = Math.max(max, Math.abs(state.speedMetersPerSecond));
        assertEquals(KinematicsConstants.MAX_ALLOWED_MODULE_VELOCITY, max, EPSILON);

        // every module scaled by the same factor: still the same motion, slower
        double factor = KinematicsConstants.MAX_ALLOWED_MODULE_VELOCITY / (omega * 2 * frontLeft.getNorm());
        ChassisSpeeds result = kinematics.toChassisSpeeds(states);
        assertEquals(speeds.vxMetersPerSecond * factor, result.vxMetersPerSecond, EPSILON);
        assertEquals(speeds.vyMetersPerSecond * factor, result.vyMetersPerSecond, EPSILON);
        assertEquals(omega * factor, result.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void zeroSpeedsGiveZeroModules() {
        for (SwerveModuleState state : kinematics.toSwerveModuleStates(new ChassisSpeeds())) {
            assertEquals(0, state.speedMetersPerSecond, EPSILON);
            assertEquals(Rotation2d.kZero.getRadians(), state.angle.getRadians(), EPSILON);
        }
    }
}
