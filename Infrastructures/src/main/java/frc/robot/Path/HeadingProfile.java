package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Reference heading theta(s) and theta'(s): piecewise linear through the anchors, or the
 * face-target heading (design doc section 6). Built per command start from the (flipped) path.
 */
final class HeadingProfile {

    /** Inside this distance of the face target the reference heading freezes [m] (6.9). */
    static final double R_FACE_MIN = 0.50;

    private HeadingProfile() {
    }

    /**
     * Builds the profile (6.13).
     *
     * @param path   the active (already flipped) path; uses table(), points(), sAtPoint, faceTarget()
     * @param theta0 measured start heading [rad], used when P_0 has no angle (6.8)
     * @return the profile
     */
    static HeadingProfile build(Path path, double theta0) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return reference heading at s [rad], unwrapped (6.3); wrap it before comparing */
    double theta(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return theta'(s) [rad/m]; 0 after the last anchor (6.4) */
    double dTheta(double s) {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return s of the piecewise-linear knots (anchors); empty array for a face target. Used for kink caps (7.4.6) */
    double[] breakpointsS() {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return true if this is the face-target heading (6.9) */
    boolean isFaceTarget() {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return the face target in the active frame, or null if none */
    Translation2d faceTarget() {
        throw new UnsupportedOperationException("TODO phase 2");
    }

    /** @return heading at the end of the path, theta(L) (8.7.2) */
    double endHeading() {
        throw new UnsupportedOperationException("TODO phase 2");
    }
}
