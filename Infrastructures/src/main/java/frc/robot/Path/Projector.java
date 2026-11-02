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

    private final SegmentTable table;

    /** @param table the geometry to project onto */
    Projector(SegmentTable table) {
        this.table = table;
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
        double length = table.length();
        double lo = Math.max(0.0, Math.min(length, sMin));
        double hi = Math.max(0.0, Math.min(length, sMax));
        if (lo > hi) {
            lo = hi;
        }
        double bestS = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;
        int first = table.segmentIndexAt(lo);
        for (int k = first; k < table.size(); k++) {
            Segment seg = table.segment(k);
            if (seg.sStart > hi) {
                break;
            }
            if (seg.sEnd() < lo) {
                continue;
            }
            double sk = Math.max(lo, Math.min(hi, projectOnSegment(k, p, sPrev)));
            double dist = p.getDistance(table.evaluate(sk).position());
            if (Double.isNaN(bestS) || dist < bestDist - SegmentTable.PROJECTION_TIE) {
                bestS = sk;
                bestDist = dist;
            } else if (Math.abs(dist - bestDist) <= SegmentTable.PROJECTION_TIE
                    && Math.abs(sk - sPrev) < Math.abs(bestS - sPrev)) {
                bestS = sk;
                bestDist = Math.min(dist, bestDist);
            }
        }
        if (Double.isNaN(bestS)) {
            bestS = lo;
        }
        PathSample sample = table.evaluate(bestS);
        Translation2d x = sample.position();
        double nx = -sample.tangent().getSin();
        double ny = sample.tangent().getCos();
        double eN = (p.getX() - x.getX()) * nx + (p.getY() - x.getY()) * ny;
        return new Projection(bestS, eN);
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
        Segment seg = table.segment(k);
        switch (seg.type) {
            case LINE: {
                double q = (p.getX() - seg.a.getX()) * seg.t.getX() + (p.getY() - seg.a.getY()) * seg.t.getY();
                return seg.sStart + Math.max(0.0, Math.min(seg.length, q));
            }
            case ARC: {
                double vx = p.getX() - seg.center.getX();
                double vy = p.getY() - seg.center.getY();
                double q;
                if (Math.hypot(vx, vy) < SegmentTable.EPS_CENTER) {
                    q = Math.max(0.0, Math.min(seg.length, sPrev - seg.sStart));
                } else {
                    double sweep = Math.abs(seg.dphi);
                    double beta = PathMath.mod2pi(seg.sigma * (Math.atan2(vy, vx) - seg.phi0));
                    if (beta <= sweep) {
                        q = seg.r * beta;
                    } else if (beta - sweep < 2.0 * Math.PI - beta) {
                        q = seg.length;
                    } else {
                        q = 0.0;
                    }
                }
                return seg.sStart + Math.max(0.0, Math.min(seg.length, q));
            }
            default:
                return seg.sStart;
        }
    }
}
