package frc.robot.Path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** NONE / MIRROR / ROTATE of a built table, points, angles and face target (design doc section 5.13). */
final class PathFlipper {

    private PathFlipper() {
    }

    /**
     * Flips a built table. s values, L and waypoint s do not change; MIRROR flips the sign of
     * sigma, kappa and dphi.
     *
     * @return a new table (or the same one for NONE)
     */
    static SegmentTable flip(SegmentTable table, FlipMode mode, double fieldLength, double fieldWidth) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a flipped position (also used for the face target and the user points) */
    static Translation2d flipTranslation(Translation2d t, FlipMode mode, double fieldLength, double fieldWidth) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a flipped heading or direction angle */
    static Rotation2d flipRotation(Rotation2d r, FlipMode mode) {
        throw new UnsupportedOperationException("TODO phase 1");
    }

    /** @return a flipped copy of a user point (position and angle; other attributes kept) */
    static Point flipPoint(Point p, FlipMode mode, double fieldLength, double fieldWidth) {
        throw new UnsupportedOperationException("TODO phase 1");
    }
}
