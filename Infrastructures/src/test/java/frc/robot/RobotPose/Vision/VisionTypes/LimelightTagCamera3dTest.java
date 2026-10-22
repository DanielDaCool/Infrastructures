package frc.robot.RobotPose.Vision.VisionTypes;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

class LimelightTagCamera3dTest {
    @Test
    void aFrameIsUsedWhenTheHeadingAgreesAndTheRobotTurnsSlowly() {
        assertTrue(LimelightTagCamera3d.isUsable(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(12), 1.0));
    }

    @Test
    void aFrameTakenWhileSpinningFastIsNotUsed() {
        assertFalse(LimelightTagCamera3d.isUsable(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(10), 2.5));
    }

    @Test
    void aFrameSolvedWithAnotherHeadingIsNotUsed() {
        assertFalse(LimelightTagCamera3d.isUsable(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(14), 0));
        // After a gyro reset to 180 deg the Limelight may still use the old heading.
        assertFalse(LimelightTagCamera3d.isUsable(Rotation2d.kZero, Rotation2d.kPi, 0));
        // Across +-180 deg.
        assertTrue(LimelightTagCamera3d.isUsable(Rotation2d.fromDegrees(179), Rotation2d.fromDegrees(-179), 0));
    }
}
