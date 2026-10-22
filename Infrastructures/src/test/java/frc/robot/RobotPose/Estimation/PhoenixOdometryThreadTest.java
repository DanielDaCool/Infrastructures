package frc.robot.RobotPose.Estimation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class PhoenixOdometryThreadTest {
    @Test
    void aValueIsMovedToTheSampleTimeWithItsRate() {
        // Yaw 90 deg measured at 1.000 s, turning 180 deg/s: at 1.004 s it is 90.72 deg.
        assertEquals(90.72, PhoenixOdometryThread.extrapolate(90, 180, 1.000, 1.004), 1e-9);
        // A value measured after the sample time is moved back.
        assertEquals(89.28, PhoenixOdometryThread.extrapolate(90, 180, 1.004, 1.000), 1e-9);
    }
}
