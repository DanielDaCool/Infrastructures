package frc.robot.Path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * One entry of the {@link SegmentTable}: LINE, ARC or STOP (design doc section 5.9.1). Plain
 * data; only the fields of its type are meaningful. Evaluation lives in {@link SegmentTable}.
 */
final class Segment {

    /** Entry type. */
    enum Type { LINE, ARC, STOP }

    final Type type;
    /** s at the start of this entry [m]. */
    final double sStart;
    /** Length [m]; 0 for STOP. */
    final double length;

    // LINE
    /** Start point A. */
    final Translation2d a;
    /** Unit direction T (LINE), or T_out (STOP). */
    final Translation2d t;

    // ARC
    /** Center C. */
    final Translation2d center;
    /** Radius r. */
    final double r;
    /** Start angle phi0 on the circle [rad]. */
    final double phi0;
    /** Signed sweep dphi [rad]. */
    final double dphi;
    /** sign(dphi): +1 left, -1 right. */
    final double sigma;
    /** s of the waypoint on this arc. */
    final double sWp;

    // STOP
    /** Direction of arrival T_in. */
    final Translation2d tIn;
    /** User index of the stop point (the index the user wrote; for merged points, the kept one). */
    final int cornerIndex;
    /** True if the user wrote stop(); false for an automatic stop corner. */
    final boolean userStop;
    /** Dwell [s]. */
    final double dwell;

    private Segment(Type type, double sStart, double length, Translation2d a, Translation2d t,
                    Translation2d center, double r, double phi0, double dphi, double sWp,
                    Translation2d tIn, int cornerIndex, boolean userStop, double dwell) {
        this.type = type;
        this.sStart = sStart;
        this.length = length;
        this.a = a;
        this.t = t;
        this.center = center;
        this.r = r;
        this.phi0 = phi0;
        this.dphi = dphi;
        this.sigma = Math.signum(dphi);
        this.sWp = sWp;
        this.tIn = tIn;
        this.cornerIndex = cornerIndex;
        this.userStop = userStop;
        this.dwell = dwell;
    }

    /** @return a LINE from a with unit direction t */
    static Segment line(double sStart, double length, Translation2d a, Translation2d t) {
        return new Segment(Type.LINE, sStart, length, a, t, null, 0, 0, 0, 0, null, -1, false, 0);
    }

    /** @return an ARC about center with radius r, start angle phi0 and signed sweep dphi */
    static Segment arc(double sStart, Translation2d center, double r, double phi0, double dphi, double sWp) {
        return new Segment(Type.ARC, sStart, r * Math.abs(dphi), null, null, center, r, phi0, dphi, sWp,
                null, -1, false, 0);
    }

    /** @return a zero-length STOP at point p arriving along tIn and leaving along tOut */
    static Segment stop(double sStart, Translation2d p, Translation2d tIn, Translation2d tOut,
                        int cornerIndex, boolean userStop, double dwell) {
        return new Segment(Type.STOP, sStart, 0, p, tOut, null, 0, 0, 0, sStart, tIn, cornerIndex,
                userStop, dwell);
    }

    /** @return s at the end of this entry */
    double sEnd() {
        return sStart + length;
    }
}
