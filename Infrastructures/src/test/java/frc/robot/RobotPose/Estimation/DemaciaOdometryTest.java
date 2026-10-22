package frc.robot.RobotPose.Estimation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

class DemaciaOdometryTest {
    private static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3) };

    private DemaciaOdometry odometry;

    @BeforeEach
    void setUp() {
        odometry = new DemaciaOdometry(MODULE_LOCATIONS);
    }

    /** All modules pointing forward with these distances. */
    private static SwerveModulePosition[] modules(double... distances) {
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModulePosition(distances[i], Rotation2d.kZero);
        }
        return modules;
    }

    @Test
    void aNaNModuleDistanceMovesNothingAndTheNextSampleCatchesUp() {
        odometry.updateOdometry(Rotation2d.kZero, modules(0, 0, 0, 0));
        odometry.updateOdometry(Rotation2d.kZero, modules(0.1, 0.1, 0.1, 0.1));
        Twist2d twist = odometry.updateOdometry(Rotation2d.kZero, modules(0.2, Double.NaN, 0.2, 0.2));
        assertEquals(0, twist.dx, 1e-12);
        odometry.updateOdometry(Rotation2d.kZero, modules(0.3, 0.3, 0.3, 0.3));
        assertEquals(0.3, odometry.getOdometryPose().getX(), 1e-12);
    }

    @Test
    void anInfiniteModuleAngleOrNaNGyroMovesNothing() {
        odometry.updateOdometry(Rotation2d.kZero, modules(0, 0, 0, 0));
        SwerveModulePosition[] bad = modules(0.1, 0.1, 0.1, 0.1);
        bad[2] = new SwerveModulePosition(0.1, new Rotation2d(Double.POSITIVE_INFINITY));
        odometry.updateOdometry(Rotation2d.kZero, bad);
        odometry.updateOdometry(new Rotation2d(Double.NaN), modules(0.1, 0.1, 0.1, 0.1));
        assertEquals(0, odometry.getOdometryPose().getX(), 1e-12);
        odometry.updateOdometry(Rotation2d.kZero, modules(0.2, 0.2, 0.2, 0.2));
        assertEquals(0.2, odometry.getOdometryPose().getX(), 1e-12);
        assertEquals(0, odometry.getOdometryPose().getRotation().getRadians(), 1e-12);
    }

    @Test
    void aRebootedEncoderIsLeftOutOfThatSample() {
        odometry.updateOdometry(Rotation2d.kZero, modules(5, 5, 5, 5));
        // Module 1 rebooted to 0 while the robot drove 0.01 m.
        odometry.updateOdometry(Rotation2d.kZero, modules(5.01, 0, 5.01, 5.01));
        assertEquals(0.01, odometry.getOdometryPose().getX(), 1e-12);
        // From then on module 1 counts from its new baseline.
        odometry.updateOdometry(Rotation2d.kZero, modules(5.02, 0.01, 5.02, 5.02));
        assertEquals(0.02, odometry.getOdometryPose().getX(), 1e-12);
    }

    @Test
    void aBigStepOnMostModulesIsStillUsed() {
        odometry.updateOdometry(Rotation2d.kZero, modules(0, 0, 0, 0));
        // Fewer than two plausible modules: nothing to compare against, so all are used.
        odometry.updateOdometry(Rotation2d.kZero, modules(0.6, 0.6, 0.6, 0.0));
        assertEquals(0.45, odometry.getOdometryPose().getX(), 1e-12);
    }
}
