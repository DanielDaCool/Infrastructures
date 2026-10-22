package frc.robot.RobotPose.Vision.VisionTypes;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import gg.questnav.questnav.PoseFrame;

class QuestTest {
    private static PoseFrame frame(double x, double timestamp, boolean tracking) {
        return new PoseFrame(new Pose3d(x, 2, 0, new Rotation3d()), timestamp, 0, 0, tracking);
    }

    @Test
    void framesAreStampedWithTheLatencyRemovedAndThinned() {
        PoseFrame[] frames = new PoseFrame[20];
        for (int i = 0; i < frames.length; i++) {
            frames[i] = frame(1, 5.0 + i * 0.01, i != 3);
        }
        List<TimestampedVisionMeasurement> measurements = Quest.toMeasurements(frames, new Transform3d(), 0.2, 0.03,
                1.0, new QuestFrameFilter(0.3, 0.2, 0.5, 0.1),
                t -> new Pose2d(1, 2, Rotation2d.kCCW_Pi_2), (from, to) -> Translation2d.kZero);
        assertEquals(2, measurements.size());
        assertEquals(4.97, measurements.get(0).timestampSeconds(), 1e-12);
        assertEquals(5.07, measurements.get(1).timestampSeconds(), 1e-12);
        assertEquals(0.2, measurements.get(0).stdDevs().get(0, 0), 1e-12);
        assertEquals(Double.POSITIVE_INFINITY, measurements.get(0).stdDevs().get(2, 0));
        assertEquals(Math.PI / 2, measurements.get(0).pose().getRotation().getRadians(), 1e-12);
    }

    @Test
    void framesBeforeTheSettleTimeAreSkipped() {
        PoseFrame[] frames = { frame(1, 1.0, true), frame(1, 1.2, true) };
        List<TimestampedVisionMeasurement> measurements = Quest.toMeasurements(frames, new Transform3d(), 0.2, 0.0,
                1.1, new QuestFrameFilter(0.3, 0.2, 0.5, 0.1), t -> Pose2d.kZero, (from, to) -> Translation2d.kZero);
        assertEquals(1, measurements.size());
        assertEquals(1.2, measurements.get(0).timestampSeconds(), 1e-12);
    }
}
