package frc.robot.Path;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.StringJoiner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Builds the {@link SegmentTable} from the user's points: merging, PASS markers, corner circles,
 * radius rule and shrink loop, tangents, sweeps (design doc sections 5.3 to 5.8, 5.15).
 */
final class GeometryBuilder {

    /** Smallest arc radius [m] (5.7.3). */
    static final double R_MIN = 0.05;
    /** Largest automatic radius [m] (5.7.2). */
    static final double R_MAX = 2.0;
    /** Bulge distance [m] (5.7.2). */
    static final double D_BULGE = 0.5;
    /** Largest allowed line deviation, 30 deg [rad] (5.7.1, F2 and F3). */
    static final double EPS_DEV = Math.toRadians(30.0);
    /** Turn above which a corner becomes a stop, 150 deg [rad] (5.6.4). */
    static final double TAU_STOP = Math.toRadians(150.0);
    /** Radius shrink factor per pass (5.7.3). */
    static final double SHRINK = 0.8;
    /** Maximum shrink passes (5.7.3). */
    static final int MAX_ITER = 1000;
    /** Largest miss of a straight point without warning W5 [m] (5.8.3). */
    static final double PASS_TOL = 0.01;
    /** Merge distance [m] (5.8.2). */
    static final double EPS_DUP = 1e-3;
    /** Straight-point angle threshold [rad] (5.3, 5.8.3). */
    static final double EPS_ANG = 1e-4;
    /** Tangent existence, zero-length segments [m] (5.4). */
    static final double EPS_LEN = 1e-6;
    /** Sweep wrap guard [rad] (5.5). */
    static final double EPS_WRAP = 1e-9;
    /** Radius divisor k for an end leg or user-stop leg (5.7.2). */
    static final double RADIUS_DIVISOR_END = 2.0;
    /** Radius divisor k for a shared leg (5.7.2). */
    static final double RADIUS_DIVISOR_SHARED = 4.0;

    /** Feasibility conditions of section 5.7.1. */
    private enum Check { F1, F2, F3 }

    /** One point after merging duplicates; attributes merged, later value wins (5.8.2). */
    private static final class Node {
        final int userIndex;
        final double x;
        final double y;
        Rotation2d angle;
        double maxVelocity;
        double radius;
        boolean stop;
        double dwell;

        Node(int userIndex, Point p) {
            this.userIndex = userIndex;
            this.x = p.x();
            this.y = p.y();
            this.angle = p.angle().orElse(null);
            this.maxVelocity = p.maxVelocity().orElse(Double.NaN);
            this.radius = p.radius().orElse(Double.NaN);
            this.stop = p.isStop();
            this.dwell = p.dwellSeconds();
        }

        boolean hasSpeed() {
            return stop || !Double.isNaN(maxVelocity);
        }

        boolean hasRadius() {
            return !Double.isNaN(radius);
        }
    }

    /** Lines and arcs of one construction pass (5.4, 5.5) with the failed checks per corner. */
    private static final class Construction {
        final boolean[] lineExists;
        final double[] qsx;
        final double[] qsy;
        final double[] tx;
        final double[] ty;
        final double[] lambda;
        final double[] arcPhi0;
        final double[] arcDphi;
        final double[] arcWin;
        final List<Set<Check>> failed;

        Construction(int n) {
            lineExists = new boolean[n - 1];
            qsx = new double[n - 1];
            qsy = new double[n - 1];
            tx = new double[n - 1];
            ty = new double[n - 1];
            lambda = new double[n - 1];
            arcPhi0 = new double[n];
            arcDphi = new double[n];
            arcWin = new double[n];
            failed = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                failed.add(EnumSet.noneOf(Check.class));
            }
        }

        boolean anyFailed() {
            for (Set<Check> f : failed) {
                if (!f.isEmpty()) {
                    return true;
                }
            }
            return false;
        }
    }

    private GeometryBuilder() {
    }

    /**
     * Runs the whole construction (section 5.15). Throws V1, V6, V30; reports W1 to W5 to warnings.
     *
     * @param points   the user's points, validated non-null
     * @param warnings receives build warnings
     * @return the table; {@code waypointS} is indexed by the user's point index
     */
    static SegmentTable build(List<Point> points, PathWarnings warnings) {
        int userCount = points.size();
        if (userCount < 2) {
            throw new IllegalArgumentException("Path needs at least 2 points (a start and an end), but you gave "
                    + userCount + ". Example: new Path(new Point(1, 2), new Point(3, 4)).");
        }
        for (int i = 0; i < userCount; i++) {
            Point p = points.get(i);
            if (p.isStop() && p.radius().isPresent() && p.radius().getAsDouble() > 0) {
                throw stopWithRadius(i, p.radius().getAsDouble());
            }
        }

        // 5.8.2: merge neighbors closer than EPS_DUP into the earlier point
        List<Node> merged = new ArrayList<>();
        int[] userToMerged = new int[userCount];
        for (int u = 0; u < userCount; u++) {
            Point p = points.get(u);
            if (!merged.isEmpty()) {
                Node kept = merged.get(merged.size() - 1);
                double d = Math.hypot(p.x() - kept.x, p.y() - kept.y);
                if (d < EPS_DUP) {
                    mergeInto(kept, u, p, d, warnings);
                    userToMerged[u] = merged.size() - 1;
                    continue;
                }
            }
            merged.add(new Node(u, p));
            userToMerged[u] = merged.size() - 1;
        }
        if (merged.size() == 1) {
            throw new IllegalArgumentException("All " + userCount + " points of the path are at the same place "
                    + "(within 1 mm), so the path has no length. Check the coordinates: two different points are "
                    + "needed, for example new Path(new Point(1, 2), new Point(3, 4)).");
        }
        for (Node node : merged) {
            if (node.stop && node.hasRadius() && node.radius > 0) {
                throw stopWithRadius(node.userIndex, node.radius);
            }
        }
        warnEndpoints(merged.get(0), merged.get(merged.size() - 1), warnings);

        // 5.8.3: set straight middle points aside as PASS markers
        List<Integer> geo = new ArrayList<>();
        for (int m = 0; m < merged.size(); m++) {
            geo.add(m);
        }
        List<Integer> passMarkers = new ArrayList<>();
        boolean changed = true;
        while (changed) {
            changed = false;
            for (int g = 1; g < geo.size() - 1; g++) {
                Node node = merged.get(geo.get(g));
                if (node.stop) {
                    continue;
                }
                Node prev = merged.get(geo.get(g - 1));
                Node next = merged.get(geo.get(g + 1));
                if (turnAngle(prev.x, prev.y, node.x, node.y, next.x, next.y) < EPS_ANG) {
                    passMarkers.add(geo.remove(g));
                    g--;
                    changed = true;
                }
            }
        }
        for (int m : passMarkers) {
            Node node = merged.get(m);
            if (node.hasRadius()) {
                warnings.add(node.userIndex, "withRadius(" + fmt(node.radius) + ") is ignored here: only a point "
                        + "where the path turns has a corner radius.");
            }
        }

        int n = geo.size();
        double[] px = new double[n];
        double[] py = new double[n];
        Node[] nodes = new Node[n];
        for (int g = 0; g < n; g++) {
            nodes[g] = merged.get(geo.get(g));
            px[g] = nodes[g].x;
            py[g] = nodes[g].y;
        }
        double[] legLength = new double[n - 1];
        double[] divisor = new double[n - 1];
        for (int j = 0; j < n - 1; j++) {
            legLength[j] = Math.hypot(px[j + 1] - px[j], py[j + 1] - py[j]);
            boolean endLeg = j == 0 || j + 1 == n - 1 || nodes[j].stop || nodes[j + 1].stop;
            divisor[j] = endLeg ? RADIUS_DIVISOR_END : RADIUS_DIVISOR_SHARED;
        }

        // 5.3: corner construction
        double[] tau = new double[n];
        double[] sigma = new double[n];
        double[] bx = new double[n];
        double[] by = new double[n];
        double[] rawSigma = new double[n];
        for (int i = 1; i < n - 1; i++) {
            double inX = (px[i] - px[i - 1]) / legLength[i - 1];
            double inY = (py[i] - py[i - 1]) / legLength[i - 1];
            double outX = (px[i + 1] - px[i]) / legLength[i];
            double outY = (py[i + 1] - py[i]) / legLength[i];
            double cr = PathMath.cross(inX, inY, outX, outY);
            tau[i] = Math.atan2(Math.abs(cr), inX * outX + inY * outY);
            rawSigma[i] = Math.signum(cr);
            double sx = -inX + outX;
            double sy = -inY + outY;
            double norm = Math.hypot(sx, sy);
            if (tau[i] > Math.PI - EPS_ANG || norm < 1e-12) {
                bx[i] = -inX;
                by[i] = -inY;
            } else {
                bx[i] = sx / norm;
                by[i] = sy / norm;
            }
        }
        for (int i = 1; i < n - 1; i++) {
            sigma[i] = rawSigma[i];
            if (!nodes[i].stop && tau[i] > Math.PI - EPS_ANG) {
                sigma[i] = uTurnSign(i, n, tau, rawSigma);
            }
        }

        // 5.7.2, 5.7.3: initial radii
        double[] r = new double[n];
        double[] r0 = new double[n];
        boolean[] override = new boolean[n];
        for (int i = 1; i < n - 1; i++) {
            Node node = nodes[i];
            if (node.stop) {
                r[i] = 0;
            } else if (node.hasRadius()) {
                override[i] = true;
                if (node.radius >= R_MIN) {
                    r[i] = node.radius;
                } else {
                    r[i] = 0;
                    if (node.radius > 0) {
                        warnings.add(node.userIndex, stopText("withRadius(" + fmt(node.radius) + ") is below "
                                + fmt(R_MIN) + " m"));
                    }
                }
            } else {
                double auto = Math.min(R_MAX, Math.min(legLength[i - 1] / divisor[i - 1], legLength[i] / divisor[i]));
                if (tau[i] >= EPS_ANG) {
                    auto = Math.min(auto, D_BULGE / (1.0 - Math.cos(tau[i] / 2.0)));
                }
                if (tau[i] > TAU_STOP) {
                    r[i] = 0;
                    warnings.add(node.userIndex, stopText("the path turns " + fmt(Math.toDegrees(tau[i]))
                            + " degrees here, more than " + fmt(Math.toDegrees(TAU_STOP)) + " degrees"));
                } else if (auto < R_MIN) {
                    r[i] = 0;
                    warnings.add(node.userIndex, stopText("the legs are too short: the largest radius that fits is "
                            + fmt(auto) + " m, less than " + fmt(R_MIN) + " m"));
                } else {
                    r[i] = auto;
                }
            }
            r0[i] = r[i];
        }

        // 5.7.3: shrink loop
        List<Set<Check>> shrinkReasons = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            shrinkReasons.add(EnumSet.noneOf(Check.class));
        }
        Construction c = construct(n, px, py, r, sigma, tau, bx, by, legLength);
        int iter = 0;
        while (c.anyFailed()) {
            if (++iter > MAX_ITER) {
                for (int i = 1; i < n - 1; i++) {
                    if (r[i] > 0) {
                        shrinkReasons.get(i).addAll(c.failed.get(i));
                        r[i] = 0;
                    }
                }
                c = construct(n, px, py, r, sigma, tau, bx, by, legLength);
                break;
            }
            boolean anyAutomatic = false;
            for (int i = 1; i < n - 1; i++) {
                if (!c.failed.get(i).isEmpty() && !override[i]) {
                    anyAutomatic = true;
                }
            }
            for (int i = 1; i < n - 1; i++) {
                if (c.failed.get(i).isEmpty() || (anyAutomatic && override[i])) {
                    continue;
                }
                shrinkReasons.get(i).addAll(c.failed.get(i));
                r[i] *= SHRINK;
                if (r[i] < R_MIN) {
                    r[i] = 0;
                }
            }
            c = construct(n, px, py, r, sigma, tau, bx, by, legLength);
        }
        for (int i = 1; i < n - 1; i++) {
            if (shrinkReasons.get(i).isEmpty()) {
                continue;
            }
            String why = checkList(shrinkReasons.get(i));
            if (r[i] == 0) {
                warnings.add(nodes[i].userIndex, stopText("radius " + fmt(r0[i]) + " m was shrunk below "
                        + fmt(R_MIN) + " m to fit, " + why));
            } else {
                warnings.add(nodes[i].userIndex, "radius " + fmt(r0[i]) + " m does not fit and was shrunk to "
                        + fmt(r[i]) + " m (" + why + "). Why: F1 = the neighbor point is too close to the circle, "
                        + "F2 = the line would run more than 30 degrees off the straight leg, F3 = the arc would "
                        + "turn much more than the corner angle. Move the points apart, or give a smaller radius.");
            }
        }

        // 5.9.1: segment table
        List<Segment> segments = new ArrayList<>();
        double[] geoS = new double[n];
        double s = 0;
        geoS[0] = 0;
        for (int j = 0; j < n - 1; j++) {
            if (j > 0) {
                int i = j;
                if (r[i] > 0) {
                    double cx = px[i] + r[i] * bx[i];
                    double cy = py[i] + r[i] * by[i];
                    Segment arc = Segment.arc(s, new Translation2d(cx, cy), r[i], c.arcPhi0[i], c.arcDphi[i],
                            s + r[i] * c.arcWin[i]);
                    segments.add(arc);
                    geoS[i] = arc.sWp;
                    s += arc.length;
                } else {
                    segments.add(Segment.stop(s, new Translation2d(px[i], py[i]),
                            new Translation2d(c.tx[j - 1], c.ty[j - 1]), new Translation2d(c.tx[j], c.ty[j]),
                            geo.get(i), nodes[i].stop, nodes[i].stop ? nodes[i].dwell : 0.0));
                    geoS[i] = s;
                }
            }
            double len = c.lambda[j] < EPS_LEN ? 0.0 : c.lambda[j];
            segments.add(Segment.line(s, len, new Translation2d(c.qsx[j], c.qsy[j]),
                    new Translation2d(c.tx[j], c.ty[j])));
            s += len;
        }
        geoS[n - 1] = s;

        // 5.9.4: s of every merged point; PASS markers by projection between their kept neighbors
        double[] mergedS = new double[merged.size()];
        int[] geoOfMerged = new int[merged.size()];
        java.util.Arrays.fill(geoOfMerged, -1);
        for (int g = 0; g < n; g++) {
            mergedS[geo.get(g)] = geoS[g];
            geoOfMerged[geo.get(g)] = g;
        }
        SegmentTable provisional = new SegmentTable(segments, new double[userCount]);
        Projector projector = new Projector(provisional);
        for (int m = 0; m < merged.size(); m++) {
            if (geoOfMerged[m] >= 0) {
                continue;
            }
            int prev = m - 1;
            while (geoOfMerged[prev] < 0) {
                prev--;
            }
            int next = m + 1;
            while (geoOfMerged[next] < 0) {
                next++;
            }
            Node node = merged.get(m);
            Translation2d p = new Translation2d(node.x, node.y);
            double sA = mergedS[prev];
            double sB = mergedS[next];
            double sPass = projector.project(p, sA, sB, sA).s();
            mergedS[m] = sPass;
            double miss = p.getDistance(provisional.evaluate(sPass).position());
            if (miss > PASS_TOL) {
                warnings.add(node.userIndex, "this point is on a straight line between its neighbors, so it is "
                        + "only a pass marker, and the path misses it by " + fmt(miss) + " m (the line next to a "
                        + "round corner is turned outward). Add .stop() to drive exactly through it, or remove the "
                        + "point and use an event at a fraction instead.");
            }
        }
        double[] waypointS = new double[userCount];
        for (int u = 0; u < userCount; u++) {
            waypointS[u] = mergedS[userToMerged[u]];
        }
        return new SegmentTable(segments, waypointS);
    }

    /** One construction pass: tangent lines (5.4), arc sweeps (5.5), checks F1 to F3 (5.7.1). */
    private static Construction construct(int n, double[] px, double[] py, double[] r, double[] sigma,
                                          double[] tau, double[] bx, double[] by, double[] legLength) {
        Construction c = new Construction(n);
        double[] cx = new double[n];
        double[] cy = new double[n];
        double[] er = new double[n];
        double[] es = new double[n];
        for (int i = 0; i < n; i++) {
            boolean circle = i > 0 && i < n - 1 && r[i] > 0;
            er[i] = circle ? r[i] : 0.0;
            es[i] = circle ? sigma[i] : 0.0;
            cx[i] = px[i] + er[i] * bx[i];
            cy[i] = py[i] + er[i] * by[i];
        }
        double[] qex = new double[n - 1];
        double[] qey = new double[n - 1];
        for (int j = 0; j < n - 1; j++) {
            double dx = cx[j + 1] - cx[j];
            double dy = cy[j + 1] - cy[j];
            double dc = Math.hypot(dx, dy);
            double h = es[j + 1] * er[j + 1] - es[j] * er[j];
            if (!(dc - Math.abs(h) > EPS_LEN)) {
                markCorners(c, j, n, er, Check.F1);
                continue;
            }
            double lam = Math.sqrt((dc - h) * (dc + h));
            double ux = dx / dc;
            double uy = dy / dc;
            double upx = -uy;
            double upy = ux;
            double nx = (h * ux + lam * upx) / dc;
            double ny = (h * uy + lam * upy) / dc;
            double tx = (lam * ux - h * upx) / dc;
            double ty = (lam * uy - h * upy) / dc;
            c.lineExists[j] = true;
            c.lambda[j] = lam;
            c.tx[j] = tx;
            c.ty[j] = ty;
            c.qsx[j] = cx[j] - es[j] * er[j] * nx;
            c.qsy[j] = cy[j] - es[j] * er[j] * ny;
            qex[j] = cx[j + 1] - es[j + 1] * er[j + 1] * nx;
            qey[j] = cy[j + 1] - es[j + 1] * er[j + 1] * ny;
            double legX = (px[j + 1] - px[j]) / legLength[j];
            double legY = (py[j + 1] - py[j]) / legLength[j];
            double deviation = Math.atan2(Math.abs(PathMath.cross(tx, ty, legX, legY)), tx * legX + ty * legY);
            if (deviation > EPS_DEV) {
                markCorners(c, j, n, er, Check.F2);
            }
        }
        for (int i = 1; i < n - 1; i++) {
            if (er[i] == 0 || !c.lineExists[i - 1] || !c.lineExists[i]) {
                continue;
            }
            double phiIn = Math.atan2(qey[i - 1] - cy[i], qex[i - 1] - cx[i]);
            double phiP = Math.atan2(py[i] - cy[i], px[i] - cx[i]);
            double phiOut = Math.atan2(c.qsy[i] - cy[i], c.qsx[i] - cx[i]);
            double wIn = PathMath.mod2pi(sigma[i] * (phiP - phiIn));
            if (wIn > 2.0 * Math.PI - EPS_WRAP) {
                wIn = 0;
            }
            double wOut = PathMath.mod2pi(sigma[i] * (phiOut - phiP));
            if (wOut > 2.0 * Math.PI - EPS_WRAP) {
                wOut = 0;
            }
            double dphi = sigma[i] * (wIn + wOut);
            c.arcPhi0[i] = phiIn;
            c.arcDphi[i] = dphi;
            c.arcWin[i] = wIn;
            if (Math.abs(Math.abs(dphi) - tau[i]) > 2.0 * EPS_DEV) {
                c.failed.get(i).add(Check.F3);
            }
        }
        return c;
    }

    /** Marks the corners with an arc at both ends of line j as failing check f. */
    private static void markCorners(Construction c, int j, int n, double[] er, Check f) {
        for (int i = j; i <= j + 1; i++) {
            if (i > 0 && i < n - 1 && er[i] > 0) {
                c.failed.get(i).add(f);
            }
        }
    }

    /** @return the U-turn turn sign: nearest proper corner, the next one first; +1 if none (5.8.4) */
    private static double uTurnSign(int i, int n, double[] tau, double[] rawSigma) {
        for (int dist = 1; dist < n; dist++) {
            for (int k : new int[] {i + dist, i - dist}) {
                if (k >= 1 && k <= n - 2 && rawSigma[k] != 0 && tau[k] <= Math.PI - EPS_ANG) {
                    return rawSigma[k];
                }
            }
        }
        return 1.0;
    }

    /** @return tau_turn at b for the polyline a, b, c, in [0, pi] (5.3) */
    private static double turnAngle(double ax, double ay, double bx, double by, double cx, double cy) {
        double inX = bx - ax;
        double inY = by - ay;
        double outX = cx - bx;
        double outY = cy - by;
        return Math.atan2(Math.abs(PathMath.cross(inX, inY, outX, outY)), inX * outX + inY * outY);
    }

    /** Merges user point p (index u) into kept, later attribute wins; warning W3 (5.8.2). */
    private static void mergeInto(Node kept, int u, Point p, double d, PathWarnings warnings) {
        StringBuilder text = new StringBuilder("points " + kept.userIndex + " and " + u + " are only "
                + fmt(d * 1000.0) + " mm apart. Point " + u + " was merged into point " + kept.userIndex
                + ". Remove one of them.");
        if (p.angle().isPresent()) {
            if (kept.angle != null) {
                text.append(both("withAngle", fmt(kept.angle.getDegrees()) + " deg",
                        fmt(p.angle().get().getDegrees()) + " deg"));
            }
            kept.angle = p.angle().get();
        }
        boolean pSpeed = p.isStop() || p.maxVelocity().isPresent();
        if (pSpeed) {
            if (kept.hasSpeed()) {
                text.append(both("a speed setting", speedText(kept.stop, kept.dwell, kept.maxVelocity),
                        speedText(p.isStop(), p.dwellSeconds(), p.maxVelocity().orElse(Double.NaN))));
            }
            kept.stop = p.isStop();
            kept.dwell = p.dwellSeconds();
            kept.maxVelocity = p.maxVelocity().orElse(Double.NaN);
        }
        if (p.radius().isPresent()) {
            if (kept.hasRadius()) {
                text.append(both("withRadius", fmt(kept.radius), fmt(p.radius().getAsDouble())));
            }
            kept.radius = p.radius().getAsDouble();
        }
        warnings.add(kept.userIndex, text.toString());
    }

    /** @return the W3 extra text for an attribute both points set */
    private static String both(String attribute, String a, String b) {
        return " both set " + attribute + " (" + a + " and " + b + "); using the later one (" + b + ").";
    }

    /** @return "stop()", "stop(1.5)" or "withVelocity(2)" */
    private static String speedText(boolean stop, double dwell, double maxVelocity) {
        if (stop) {
            return dwell > 0 ? "stop(" + fmt(dwell) + ")" : "stop()";
        }
        return "withVelocity(" + fmt(maxVelocity) + ")";
    }

    /** Warning W4 for settings on the first and last point that have no effect (9.12.2). */
    private static void warnEndpoints(Node first, Node last, PathWarnings warnings) {
        if (first.stop) {
            if (first.dwell > 0) {
                warnings.add(first.userIndex, "stop(" + fmt(first.dwell) + ") on the first point does not change "
                        + "the path; the robot waits " + fmt(first.dwell) + " s before it starts.");
            } else {
                warnings.add(first.userIndex, "stop() is ignored here: the robot starts with its measured speed.");
            }
        } else if (!Double.isNaN(first.maxVelocity)) {
            warnings.add(first.userIndex, "withVelocity(" + fmt(first.maxVelocity) + ") is ignored here: the robot "
                    + "starts with its measured speed.");
        }
        if (first.hasRadius()) {
            warnings.add(first.userIndex, "withRadius(" + fmt(first.radius) + ") is ignored here: only a point where "
                    + "the path turns has a corner radius.");
        }
        if (!Double.isNaN(last.maxVelocity)) {
            warnings.add(last.userIndex, "withVelocity(" + fmt(last.maxVelocity) + ") is ignored here: the robot "
                    + "always ends at speed 0 in this version.");
        }
        if (last.hasRadius()) {
            warnings.add(last.userIndex, "withRadius(" + fmt(last.radius) + ") is ignored here: only a point where "
                    + "the path turns has a corner radius.");
        }
    }

    /** @return the W1 text with its reason */
    private static String stopText(String reason) {
        return "the corner is too sharp or too tight for an arc (" + reason + "). The robot will stop here. To drive "
                + "through it with an arc, give a radius of at least " + fmt(R_MIN) + " m, for example "
                + "withRadius(0.3). To stop on purpose, use stop() or withRadius(0).";
    }

    /** @return "F1, F2" style list */
    private static String checkList(Set<Check> checks) {
        StringJoiner joiner = new StringJoiner(", ");
        for (Check check : checks) {
            joiner.add(check.name());
        }
        return joiner.toString();
    }

    /** @return the V30 exception for point i */
    private static IllegalArgumentException stopWithRadius(int i, double radius) {
        return new IllegalArgumentException("Point " + i + " has both stop() and withRadius(" + radius + "). A stop "
                + "point is a sharp corner (the robot stops exactly on the point), so it has no arc. Remove withRadius("
                + radius + ") to stop on the point, or remove stop() to drive through the corner on a round arc.");
    }

    /** @return v rounded to 4 decimals, without trailing zeros */
    static String fmt(double v) {
        if (!Double.isFinite(v)) {
            return Double.toString(v);
        }
        return BigDecimal.valueOf(v).setScale(4, RoundingMode.HALF_UP).stripTrailingZeros().toPlainString();
    }
}
