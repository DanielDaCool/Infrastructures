package frc.robot.RobotPose.Vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotPose.Vision.visionConfigs.LimelightTagCamera2dConfig;

class BaseVisionSourceTest {
    /** A source with no device behind it. */
    private static final class TestSource extends BaseVisionSource {
        TestSource(String name) {
            super(new LimelightTagCamera2dConfig(name, new Transform3d(), VecBuilder.fill(0.02, 0.02, 1)));
        }

        @Override
        public boolean shouldUpdate() {
            return false;
        }

        @Override
        public List<TimestampedVisionMeasurement> getPoseEstimates() {
            return List.of();
        }

        @Override
        public boolean isConnected() {
            return false;
        }
    }

    @Test
    void theCallbackGetsEarlierAndLaterSources() {
        HAL.initialize(500, 0);
        VisionSource before = new TestSource("before");
        List<VisionSource> registered = new ArrayList<>();
        BaseVisionSource.setOnCreate(registered::add);
        VisionSource after = new TestSource("after");
        assertTrue(registered.contains(before));
        assertTrue(registered.contains(after));
        BaseVisionSource.setOnCreate(source -> { });
    }

    @Test
    void theStdDevGrowsWithDistanceAndShrinksWithTags() {
        // Close: the minimum.
        assertEquals(0.02, BaseVisionSource.distanceStdDev(0.02, 1.0, 1), 1e-12);
        // 4 m, 1 tag: 0.008 * 16 = 0.128, heading part 0.04.
        assertEquals(Math.hypot(0.128, 0.04), BaseVisionSource.distanceStdDev(0.02, 4.0, 1), 1e-12);
        // 4 tags halve the distance part.
        assertEquals(Math.hypot(0.064, 0.04), BaseVisionSource.distanceStdDev(0.02, 4.0, 4), 1e-12);
        // No tag count reported counts as one.
        assertEquals(BaseVisionSource.distanceStdDev(0.02, 4.0, 1), BaseVisionSource.distanceStdDev(0.02, 4.0, 0), 1e-12);
    }
}
