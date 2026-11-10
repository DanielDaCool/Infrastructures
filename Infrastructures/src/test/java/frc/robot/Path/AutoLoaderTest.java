package frc.robot.Path;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.junit.jupiter.api.Test;

class AutoLoaderTest {
    @Test
    void readsPathNamesInOrder() {
        assertEquals(List.of("A", "B", "A"), AutoLoader.fromJson("{ \"version\": 1, \"paths\": [ \"A\", \"B\", \"A\" ] }"));
        assertEquals(List.of(), AutoLoader.fromJson("{ \"version\": 1, \"paths\": [] }"));
    }

    @Test
    void rejectsBadFiles() {
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson("not json"));
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson("{ \"paths\": [] }")); // no version
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson("{ \"version\": 2, \"paths\": [] }"));
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson("{ \"version\": 1 }"));
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson("{ \"version\": 1, \"paths\": [ 3 ] }"));
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson("{ \"version\": 1, \"paths\": [ \"\" ] }"));
        assertThrows(IllegalArgumentException.class, () -> AutoLoader.fromJson(
                "{ \"version\": 1, \"path\": [ \"A\" ], \"paths\": [] }")); // typo
    }
}
