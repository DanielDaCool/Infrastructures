package frc.robot.Path;

import java.util.List;

/**
 * The built geometry: LINE / ARC / STOP entries in order, with arc-length evaluation (design doc
 * section 5.9). Immutable after construction. Shared by {@link Path}, {@link Projector},
 * {@link PathFlipper}, {@link HeadingProfile}, {@link SpeedProfile} and {@link Tracker}.
 */
final class SegmentTable {

    /** Robot at an arc center: every arc point is equally near (section 10, EPS_CENTER). */
    static final double EPS_CENTER = 1e-9;
    /** Projection tie distance (section 10, 5.10.4). */
    static final double PROJECTION_TIE = 1e-12;

    /**
     * @param segments  entries in order, line, corner, line, ... (section 5.9.1)
     * @param waypointS s of each point the user wrote, by user index (merged points map to the kept one)
     */
    SegmentTable(List<Segment> segments, double[] waypointS) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return total length L [m] (section 5.9.1) */
    double length() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return number of entries m */
    int size() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return entry k */
    Segment segment(int k) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return index of the entry that contains s, later entry at a boundary, never zero-length (5.9.3) */
    int segmentIndexAt(double s) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the sample at s clamped to [0, L]; T is the right limit at a STOP (5.9.2, 5.9.3) */
    PathSample evaluate(double s) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return the sample at s using the largest positive-length entry with sStart &lt; s: T of the leg arrived on (5.9.3) */
    PathSample evaluateLeft(double s) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return s of the point the user wrote at index i (5.9.4) */
    double waypointS(int i) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return number of user points */
    int waypointCount() {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return s of every STOP entry that is a stop point (userStop or automatic), ascending, excluding s = 0 and L (7.10, 8.6) */
    double[] stopS() {
        throw new UnsupportedOperationException("TODO phase 1");
    }
}
