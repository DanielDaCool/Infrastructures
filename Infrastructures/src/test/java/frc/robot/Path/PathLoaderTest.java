package frc.robot.Path;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
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
}
