package frc.robot.RobotPose.Vision.VisionTypes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.DoubleFunction;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class QuestFrameFilterTest {
    private QuestFrameFilter filter;
    /** The robot stands still at (1, 1) according to the estimate and the odometry. */
    private final DoubleFunction<Translation2d> estimate = t -> new Translation2d(1, 1);
    private final QuestFrameFilter.OdometryMotion noMotion = (from, to) -> Translation2d.kZero;

    @BeforeEach
    void setUp() {
        filter = new QuestFrameFilter(0.3, 0.2, 0.5, 0.1);
    }

    @Test
    void framesAreThinnedToOnePerPeriod() {
        int used = 0;
        for (int i = 0; i < 100; i++) {  // 1 s at 100 Hz
            if (filter.filter(i * 0.01, new Translation2d(1, 1), estimate, noMotion) != null) {
                used++;
            }
        }
        assertEquals(10, used);
    }

    @Test
    void aJumpIsIgnoredThenRealignedToTheEstimate() {
        assertNotNull(filter.filter(0.0, new Translation2d(1, 1), estimate, noMotion));
        // The Quest jumps 0.5 m while the odometry says the robot didn't move.
        assertNull(filter.filter(0.1, new Translation2d(1.5, 1), estimate, noMotion));
        for (int i = 2; i <= 6; i++) {
            assertNull(filter.filter(i * 0.1, new Translation2d(1.5, 1), estimate, noMotion));
        }
        // 0.6 s: re-aligned on this frame (not used), so the Quest's 1.5 now means 1.
        assertNull(filter.filter(0.6, new Translation2d(1.5, 1), estimate, noMotion));
        Translation2d used = filter.filter(0.7, new Translation2d(1.6, 1), estimate, noMotion);
        assertEquals(1.1, used.getX(), 1e-12);
        assertEquals(1.0, used.getY(), 1e-12);
    }

    @Test
    void motionThatMatchesTheOdometryIsNotAJump() {
        QuestFrameFilter.OdometryMotion driving = (from, to) -> new Translation2d(4 * (to - from), 0);
        int used = 0;
        for (int i = 0; i <= 20; i++) {
            if (filter.filter(i * 0.1, new Translation2d(1 + 0.4 * i, 1), estimate, driving) != null) {
                used++;
            }
        }
        assertEquals(21, used);
    }

    @Test
    void lostTrackingIsTreatedLikeAJump() {
        assertNotNull(filter.filter(0.0, new Translation2d(1, 1), estimate, noMotion));
        // No frame for 0.3 s.
        assertNull(filter.filter(0.3, new Translation2d(1, 1), estimate, noMotion));
        for (int i = 4; i <= 7; i++) {
            assertNull(filter.filter(i * 0.1, new Translation2d(1, 1), estimate, noMotion));
        }
        // After the 0.5 s blackout: re-aligned on this frame, used from the next period.
        assertNull(filter.filter(0.85, new Translation2d(1, 1), estimate, noMotion));
        assertNotNull(filter.filter(0.95, new Translation2d(1, 1), estimate, noMotion));
    }

    @Test
    void theOffsetCanBeShiftedAndReset() {
        filter.shiftOffset(new Translation2d(0.2, -0.1));
        Translation2d used = filter.filter(0.0, new Translation2d(1, 1), estimate, noMotion);
        assertEquals(1.2, used.getX(), 1e-12);
        assertEquals(0.9, used.getY(), 1e-12);
        filter.reset();
        assertEquals(Translation2d.kZero, filter.getOffset());
    }
}
