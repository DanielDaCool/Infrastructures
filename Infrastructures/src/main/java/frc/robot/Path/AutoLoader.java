package frc.robot.Path;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Loads an auto saved by the path designer from {@code deploy/autos/<name>.json}: the paths to drive, in order.
 *
 * <pre>{@code
 * { "version": 1, "paths": [ "StartToNote", "NoteToShoot" ] }
 * }</pre>
 *
 * Each name is a file in deploy/paths, loaded with {@link PathLoader#load(String)}.
 * Load in the constructor or robotInit, not in auto: a bad file throws.
 */
public final class AutoLoader {
    private AutoLoader() {}

    public static final int VERSION = 1;
    private static final ObjectMapper MAPPER = new ObjectMapper();

    /** @param name file name in deploy/autos, without ".json" */
    public static List<Path> load(String name) {
        File file = new File(Filesystem.getDeployDirectory(), "autos/" + name + ".json");
        List<String> names;
        try {
            names = fromJson(Files.readString(file.toPath()));
        } catch (IOException e) {
            throw new IllegalArgumentException("Cannot read auto file " + file + ": " + e.getMessage(), e);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("Auto file " + file + ": " + e.getMessage(), e);
        }
        List<Path> paths = new ArrayList<>();
        for (String path : names) paths.add(PathLoader.load(path));
        return paths;
    }

    /** The path names in the JSON text of an auto file, in order. */
    public static List<String> fromJson(String json) {
        JsonNode root;
        try {
            root = MAPPER.readTree(json);
        } catch (IOException e) {
            throw new IllegalArgumentException("not valid JSON: " + e.getMessage(), e);
        }
        if (root == null || !root.isObject()) throw new IllegalArgumentException("not a JSON object");
        int version = root.path("version").asInt(-1);
        if (version != VERSION) throw new IllegalArgumentException("version " + version + ", expected " + VERSION);
        for (Iterator<String> fields = root.fieldNames(); fields.hasNext(); ) {
            String field = fields.next();
            if (!field.equals("version") && !field.equals("paths"))
                throw new IllegalArgumentException("unknown field \"" + field + "\"");
        }
        JsonNode list = root.path("paths");
        if (!list.isArray()) throw new IllegalArgumentException("missing \"paths\" array");
        List<String> names = new ArrayList<>();
        for (int i = 0; i < list.size(); i++) {
            JsonNode name = list.get(i);
            if (!name.isTextual() || name.asText().isEmpty())
                throw new IllegalArgumentException("path " + i + " is not a file name");
            names.add(name.asText());
        }
        return List.copyOf(names);
    }
}
