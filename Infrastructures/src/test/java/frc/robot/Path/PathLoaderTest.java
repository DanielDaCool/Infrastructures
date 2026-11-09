package frc.robot.Path;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class PathLoaderTest {
    @Test
    void loadsSameGeometryAsCode() {
        Path fromFile = PathLoader.fromJson("""
                { "version": 1, "points": [
                  { "x": 0, "y": 0 },
                  { "x": 2, "y": 0, "angleDeg": 90, "radius": 0.4 },
                  { "x": 2, "y": 2, "stop": true, "dwell": 0.5 },
                  { "x": 4, "y": 2 } ] }
                """);
        Path fromCode = new Path(new Point(0, 0), new Point(2, 0).withAngle(Rotation2d.fromDegrees(90)).withRadius(0.4),
                new Point(2, 2).stop(0.5), new Point(4, 2));
        assertEquals(fromCode.length(), fromFile.length(), 1e-12);
        assertEquals(fromCode.stops(), fromFile.stops());
        assertEquals(fromCode.anchors().size(), fromFile.anchors().size());
        assertEquals(Math.PI / 2, fromFile.anchors().get(0).angle().getRadians(), 1e-12);
    }

    @Test
    void rejectsBadFiles() {
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson("not json"));
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson("{ \"points\": [] }")); // no version
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson("{ \"version\": 2, \"points\": [] }"));
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"points\": [ { \"x\": 0 }, { \"x\": 1, \"y\": 1 } ] }")); // missing y
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"points\": [ { \"x\": 0, \"y\": 0, \"angle\": 9 }, { \"x\": 1, \"y\": 1 } ] }")); // typo
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"points\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 1, \"y\": 1, \"dwell\": 1 } ] }"));
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"points\": [ { \"x\": 0, \"y\": 0 } ] }")); // one point: Path rejects it
    }

    @Test
    void limitsOverrideOnlyWhatIsGiven() {
        String points = "\"points\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 3, \"y\": 0 } ]";
        Path defaults = PathLoader.fromJson("{ \"version\": 1, " + points + " }");
        assertEquals(PathConfig.MAX_VELOCITY, defaults.maxVelocity());
        Path custom = PathLoader.fromJson("{ \"version\": 1, \"limits\": { \"maxVelocity\": 0.5, \"maxOmegaDeg\": 90 }, " + points + " }");
        assertEquals(0.5, custom.maxVelocity());
        assertEquals(PathConfig.MAX_ACCELERATION, custom.maxAcceleration());
        assertEquals(Math.PI / 2, custom.maxOmega(), 1e-12);
        assertEquals(0.5, custom.flipped(true).maxVelocity()); // red keeps the limits
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"limits\": { \"maxVelocity\": 0 }, " + points + " }"));
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"limits\": { \"maxSpeed\": 1 }, " + points + " }")); // typo
        assertThrows(IllegalArgumentException.class, () -> PathLoader.fromJson(
                "{ \"version\": 1, \"limit\": { \"maxVelocity\": 1 }, " + points + " }")); // typo at the top
    }

    @Test
    void followerUsesThePathLimit() {
        Path path = new Path(new Point(0, 0), new Point(6, 0)).withLimits(0.5, PathConfig.MAX_ACCELERATION, PathConfig.MAX_OMEGA);
        PathFollower follower = new PathFollower(path, new Pose2d(0, 0, new Rotation2d(0)));
        double x = 0, top = 0;
        for (int i = 0; i < 200; i++) { // 4 s, a point robot that drives exactly the command
            ChassisSpeeds c = follower.calculate(new Pose2d(x, 0, new Rotation2d(0)), new ChassisSpeeds(top, 0, 0));
            top = Math.max(top, c.vxMetersPerSecond);
            x += c.vxMetersPerSecond * 0.02;
        }
        assertEquals(0.5, top, 1e-9);
    }
}
