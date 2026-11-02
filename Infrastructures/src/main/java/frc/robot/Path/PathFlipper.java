package frc.robot.Path;

import java.util.ArrayList;
import java.util.List;

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
        if (mode == FlipMode.NONE) {
            return table;
        }
        List<Segment> out = new ArrayList<>(table.size());
        for (int k = 0; k < table.size(); k++) {
            Segment seg = table.segment(k);
            switch (seg.type) {
                case LINE:
                    out.add(Segment.line(seg.sStart, seg.length,
                            flipTranslation(seg.a, mode, fieldLength, fieldWidth), flipDirection(seg.t, mode)));
                    break;
                case ARC: {
                    double phi0 = mode == FlipMode.MIRROR ? Math.PI - seg.phi0 : seg.phi0 + Math.PI;
                    double dphi = mode == FlipMode.MIRROR ? -seg.dphi : seg.dphi;
                    out.add(Segment.arc(seg.sStart, flipTranslation(seg.center, mode, fieldLength, fieldWidth),
                            seg.r, PathMath.wrap(phi0), dphi, seg.sWp));
                    break;
                }
                default:
                    out.add(Segment.stop(seg.sStart, flipTranslation(seg.a, mode, fieldLength, fieldWidth),
                            flipDirection(seg.tIn, mode), flipDirection(seg.t, mode), seg.cornerIndex,
                            seg.userStop, seg.dwell));
                    break;
            }
        }
        double[] waypointS = new double[table.waypointCount()];
        for (int i = 0; i < waypointS.length; i++) {
            waypointS[i] = table.waypointS(i);
        }
        return new SegmentTable(out, waypointS);
    }

    /** @return a flipped position (also used for the face target and the user points) */
    static Translation2d flipTranslation(Translation2d t, FlipMode mode, double fieldLength, double fieldWidth) {
        switch (mode) {
            case MIRROR:
                return new Translation2d(fieldLength - t.getX(), t.getY());
            case ROTATE:
                return new Translation2d(fieldLength - t.getX(), fieldWidth - t.getY());
            default:
                return t;
        }
    }

    /** @return a flipped heading or direction angle */
    static Rotation2d flipRotation(Rotation2d r, FlipMode mode) {
        switch (mode) {
            case MIRROR:
                return new Rotation2d(-r.getCos(), r.getSin());
            case ROTATE:
                return new Rotation2d(-r.getCos(), -r.getSin());
            default:
                return r;
        }
    }

    /** @return a flipped copy of a user point (position and angle; other attributes kept) */
    static Point flipPoint(Point p, FlipMode mode, double fieldLength, double fieldWidth) {
        if (mode == FlipMode.NONE) {
            return p;
        }
        Translation2d pos = flipTranslation(p.position(), mode, fieldLength, fieldWidth);
        Rotation2d angle = p.angle().map(a -> flipRotation(a, mode)).orElse(null);
        return p.withPositionAndAngle(pos.getX(), pos.getY(), angle);
    }

    /** @return a flipped unit direction vector (MIRROR: (-x, y); ROTATE: -T) */
    private static Translation2d flipDirection(Translation2d t, FlipMode mode) {
        if (t == null) {
            return null;
        }
        switch (mode) {
            case MIRROR:
                return new Translation2d(-t.getX(), t.getY());
            case ROTATE:
                return new Translation2d(-t.getX(), -t.getY());
            default:
                return t;
        }
    }
}
