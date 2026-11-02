package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/** Windowed projection of the robot position onto the path (design doc sections 5.10, 8.2.3). */
final class Projector {

    /**
     * Result of a projection.
     *
     * @param s  s* [m]
     * @param eN signed cross-track error, + = left of the path [m]
     */
    record Projection(double s, double eN) {
    }

    /** @param table the geometry to project onto */
    Projector(SegmentTable table) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /**
     * Nearest path point with s in [sMin, sMax]; ties keep the s closest to sPrev (5.10.4).
     *
     * @param p     robot position
     * @param sMin  window start
     * @param sMax  window end
     * @param sPrev previous s*
     * @return s* and e_n
     */
    Projection project(Translation2d p, double sMin, double sMax, double sPrev) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /**
     * Projection onto one entry, unclamped to the window (5.10.1, 5.10.2, 5.10.3).
     *
     * @param k     entry index
     * @param p     robot position
     * @param sPrev previous s* (used when p is at an arc center)
     * @return s on entry k
     */
    double projectOnSegment(int k, Translation2d p, double sPrev) {
        throw new UnsupportedOperationException("TODO phase 1");
    }
}
