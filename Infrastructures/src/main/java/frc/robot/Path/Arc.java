package frc.robot.Path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/** Circular segment around a center, turning CCW (sigma = +1) or CW (sigma = -1). */
public final class Arc extends Segment {
    private final Translation2d center;
    private final double radius;
    private final int sigma;
    private final double startAngle;
    private final double sweep;

    /**
     * @param center circle center [m]
     * @param r radius [m], > 0
     * @param sigma +1 CCW, -1 CW
     * @param startAngle polar angle of the start point around the center [rad]
     * @param sweep turned angle [rad], > 0
     */
    public Arc(Translation2d center, double r, int sigma, double startAngle, double sweep) {
        if (!(r > 0) || !(sweep > 0) || Math.abs(sigma) != 1) throw new IllegalArgumentException("Bad arc");
        this.center = center;
        this.radius = r;
        this.sigma = sigma;
        this.startAngle = startAngle;
        this.sweep = sweep;
    }

    public Translation2d center() { return center; }

    /** Radius [m]. */
    public double radius() { return radius; }

    /** +1 CCW, -1 CW. */
    public int sigma() { return sigma; }

    @Override
    public double length() { return radius * sweep; }

    @Override
    public Translation2d pointAt(double t) {
        double phi = startAngle + sigma * t / radius;
        return center.plus(new Translation2d(radius * Math.cos(phi), radius * Math.sin(phi)));
    }

    @Override
    public Translation2d tangentAt(double t) {
        double phi = startAngle + sigma * t / radius;
        return new Translation2d(-sigma * Math.sin(phi), sigma * Math.cos(phi));
    }

    @Override
    public double project(Translation2d p) {
        Translation2d rel = p.minus(center);
        double phi = Math.atan2(rel.getY(), rel.getX());
        return radius * (MathUtil.angleModulus(sigma * (phi - startAngle) - sweep / 2) + sweep / 2);
    }

    @Override
    public double curvature() { return sigma / radius; }
}
