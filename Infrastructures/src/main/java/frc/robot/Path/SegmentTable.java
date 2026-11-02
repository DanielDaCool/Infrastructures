package frc.robot.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
    /** A STOP closer than this to 0 or L is not reported by {@link #stopS()} [m]. */
    private static final double STOP_END_TOLERANCE = 1e-9;

    private final List<Segment> segments;
    private final double[] sStarts;
    private final double[] waypointS;
    private final double length;

    /**
     * @param segments  entries in order, line, corner, line, ... (section 5.9.1)
     * @param waypointS s of each point the user wrote, by user index (merged points map to the kept one)
     */
    SegmentTable(List<Segment> segments, double[] waypointS) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException("SegmentTable needs at least one segment, but it got none. Give the path"
                    + " two points that are not at the same place, for example new Path(new Point(1, 2),"
                    + " new Point(3, 4)).");
        }
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));
        this.waypointS = waypointS.clone();
        this.sStarts = new double[segments.size()];
        for (int k = 0; k < segments.size(); k++) {
            sStarts[k] = segments.get(k).sStart;
        }
        this.length = segments.get(segments.size() - 1).sEnd();
    }

    /** @return total length L [m] (section 5.9.1) */
    double length() {
        return length;
    }

    /** @return number of entries m */
    int size() {
        return segments.size();
    }

    /** @return entry k */
    Segment segment(int k) {
        return segments.get(k);
    }

    /** @return index of the entry that contains s, later entry at a boundary, never zero-length (5.9.3) */
    int segmentIndexAt(double s) {
        double sc = clampS(s);
        int k = largestStartAtOrBefore(sc);
        while (k > 0 && segments.get(k).length == 0) {
            k--;
        }
        return k;
    }

    /** @return the sample at s clamped to [0, L]; T is the right limit at a STOP (5.9.2, 5.9.3) */
    PathSample evaluate(double s) {
        double sc = clampS(s);
        return evaluateOn(segmentIndexAt(sc), sc);
    }

    /** @return the sample at s using the largest positive-length entry with sStart &lt; s: T of the leg arrived on (5.9.3) */
    PathSample evaluateLeft(double s) {
        double sc = clampS(s);
        int k = segments.size() - 1;
        while (k > 0 && (segments.get(k).length == 0 || sStarts[k] >= sc)) {
            k--;
        }
        return evaluateOn(k, sc);
    }

    /** @return s of the point the user wrote at index i (5.9.4) */
    double waypointS(int i) {
        return waypointS[i];
    }

    /** @return number of user points */
    int waypointCount() {
        return waypointS.length;
    }

    /** @return s of every STOP entry that is a stop point (userStop or automatic), ascending, excluding s = 0 and L (7.10, 8.6) */
    double[] stopS() {
        List<Double> stops = new ArrayList<>();
        for (Segment seg : segments) {
            if (seg.type == Segment.Type.STOP && seg.sStart > STOP_END_TOLERANCE
                    && seg.sStart < length - STOP_END_TOLERANCE) {
                stops.add(seg.sStart);
            }
        }
        double[] out = new double[stops.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = stops.get(i);
        }
        return out;
    }

    /** @return the unit tangent T on entry k at s (s clamped to the entry) */
    Translation2d tangentOn(int k, double s) {
        Segment seg = segments.get(k);
        switch (seg.type) {
            case ARC: {
                double phi = arcAngle(seg, s);
                return new Translation2d(-seg.sigma * Math.sin(phi), seg.sigma * Math.cos(phi));
            }
            default:
                return seg.t;
        }
    }

    /** @return the point x(s) on entry k (s clamped to the entry) */
    Translation2d positionOn(int k, double s) {
        Segment seg = segments.get(k);
        switch (seg.type) {
            case LINE: {
                double q = clampQ(seg, s);
                return new Translation2d(seg.a.getX() + q * seg.t.getX(), seg.a.getY() + q * seg.t.getY());
            }
            case ARC: {
                double phi = arcAngle(seg, s);
                return new Translation2d(seg.center.getX() + seg.r * Math.cos(phi),
                        seg.center.getY() + seg.r * Math.sin(phi));
            }
            default:
                return seg.a;
        }
    }

    /** @return the sample of entry k at s (s clamped to the entry) */
    private PathSample evaluateOn(int k, double s) {
        Segment seg = segments.get(k);
        Translation2d t = tangentOn(k, s);
        double curvature = seg.type == Segment.Type.ARC ? seg.sigma / seg.r : 0.0;
        return new PathSample(s, positionOn(k, s), new Rotation2d(t.getX(), t.getY()), curvature);
    }

    /** @return phi on an arc at s: phi0 + sigma * q / r */
    private static double arcAngle(Segment seg, double s) {
        return seg.phi0 + seg.sigma * clampQ(seg, s) / seg.r;
    }

    /** @return q = s - sStart clamped to [0, l] */
    private static double clampQ(Segment seg, double s) {
        return Math.max(0.0, Math.min(seg.length, s - seg.sStart));
    }

    /** @return s clamped to [0, L]; NaN becomes 0 */
    private double clampS(double s) {
        if (Double.isNaN(s)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(length, s));
    }

    /** @return the largest index with sStart &lt;= s (binary search; 0 if none) */
    int largestStartAtOrBefore(double s) {
        int lo = 0;
        int hi = sStarts.length - 1;
        int result = 0;
        while (lo <= hi) {
            int mid = (lo + hi) >>> 1;
            if (sStarts[mid] <= s) {
                result = mid;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }
        return result;
    }
}
