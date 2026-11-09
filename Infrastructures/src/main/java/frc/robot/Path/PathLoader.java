package frc.robot.Path;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Iterator;
import java.util.Set;

/**
 * Loads a path saved by the path designer from {@code deploy/paths/<name>.json}.
 *
 * <pre>{@code
 * { "version": 1,
 *   "points": [ { "x": 1.0, "y": 2.0 },
 *               { "x": 3.0, "y": 2.0, "angleDeg": 90, "radius": 0.4 },
 *               { "x": 3.0, "y": 4.0, "stop": true, "dwell": 0.5 } ] }
 * }</pre>
 *
 * x, y [m] blue field coordinates; angleDeg robot heading [deg]; radius corner radius [m];
 * stop and dwell [s] as in {@link Point#stop(double)}. Optional fields may be left out.
 * Load in the constructor or robotInit, not in auto: a bad file throws.
 */
public final class PathLoader {
    private PathLoader() {}

    public static final int VERSION = 1;
    private static final ObjectMapper MAPPER = new ObjectMapper();
    private static final Set<String> POINT_FIELDS = Set.of("x", "y", "angleDeg", "radius", "stop", "dwell");

    /** @param name file name in deploy/paths, without ".json" */
    public static Path load(String name) {
        File file = new File(Filesystem.getDeployDirectory(), "paths/" + name + ".json");
        try {
            return fromJson(Files.readString(file.toPath()));
        } catch (IOException e) {
            throw new IllegalArgumentException("Cannot read path file " + file + ": " + e.getMessage(), e);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("Path file " + file + ": " + e.getMessage(), e);
        }
    }

    /** Builds a path from the JSON text of a path file. */
    public static Path fromJson(String json) {
        JsonNode root;
        try {
            root = MAPPER.readTree(json);
        } catch (IOException e) {
            throw new IllegalArgumentException("not valid JSON: " + e.getMessage(), e);
        }
        if (root == null || !root.isObject()) throw new IllegalArgumentException("not a JSON object");
        int version = root.path("version").asInt(-1);
        if (version != VERSION) throw new IllegalArgumentException("version " + version + ", expected " + VERSION);
        JsonNode list = root.path("points");
        if (!list.isArray()) throw new IllegalArgumentException("missing \"points\" array");

        Point[] points = new Point[list.size()];
        for (int i = 0; i < points.length; i++) points[i] = point(list.get(i), i);
        return new Path(points);
    }

    private static Point point(JsonNode node, int index) {
        if (!node.isObject()) throw new IllegalArgumentException("point " + index + " is not an object");
        for (Iterator<String> names = node.fieldNames(); names.hasNext(); ) {
            String field = names.next();
            if (!POINT_FIELDS.contains(field))
                throw new IllegalArgumentException("point " + index + ": unknown field \"" + field + "\"");
        }
        Point point = new Point(number(node, "x", index), number(node, "y", index));
        if (node.has("angleDeg")) point = point.withAngle(Rotation2d.fromDegrees(number(node, "angleDeg", index)));
        if (node.has("radius")) point = point.withRadius(number(node, "radius", index));
        if (node.has("stop")) {
            if (!node.get("stop").isBoolean()) throw new IllegalArgumentException("point " + index + ": \"stop\" is not true/false");
            if (node.get("stop").asBoolean()) point = point.stop(node.has("dwell") ? number(node, "dwell", index) : 0);
        }
        if (node.has("dwell") && !point.isStop())
            throw new IllegalArgumentException("point " + index + ": \"dwell\" without \"stop\": true");
        return point;
    }

    private static double number(JsonNode node, String field, int index) {
        JsonNode value = node.get(field);
        if (value == null || !value.isNumber())
            throw new IllegalArgumentException("point " + index + ": \"" + field + "\" is missing or not a number");
        return value.asDouble();
    }
}
