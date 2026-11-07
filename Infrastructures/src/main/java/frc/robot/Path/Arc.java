package frc.robot.Path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/** Circular segment around a center, turning left (CCW) or right (CW). */
public final class Arc extends Segment {
    private final Translation2d center;
    private final double radius;
    private final boolean isLeftTurn;
    private final double turn; // +1 left, -1 right: the sign used in the formulas
    private final double startAngle;
    private final double sweep;

    /**
     * @param center circle center [m]
     * @param r radius [m], > 0
     * @param isLeftTurn true for a left (CCW) turn, false for a right (CW) turn
     * @param startAngle polar angle of the start point around the center [rad]
     * @param sweep turned angle [rad], > 0
     */
    public Arc(Translation2d center, double r, boolean isLeftTurn, double startAngle, double sweep) {
        if (!(r > 0) || !(sweep > 0)) throw new IllegalArgumentException("Bad arc");
        this.center = center;
        this.radius = r;
        this.isLeftTurn = isLeftTurn;
        this.turn = isLeftTurn ? 1 : -1;
        this.startAngle = startAngle;
        this.sweep = sweep;
    }

    /** True for a left (CCW) turn. */
    public boolean isLeftTurn() { return isLeftTurn; }

    @Override
    public double length() { return radius * sweep; }

    @Override
    public Translation2d pointAt(double t) {
        double phi = startAngle + turn * t / radius;
        return center.plus(new Translation2d(radius * Math.cos(phi), radius * Math.sin(phi)));
    }

    @Override
    public Translation2d tangentAt(double t) {
        double phi = startAngle + turn * t / radius;
        return new Translation2d(-turn * Math.sin(phi), turn * Math.cos(phi));
    }

    @Override
    public double project(Translation2d p) {
        Translation2d rel = p.minus(center);
        double phi = Math.atan2(rel.getY(), rel.getX());
        return radius * (MathUtil.angleModulus(turn * (phi - startAngle) - sweep / 2) + sweep / 2);
    }

    @Override
    public double curvature() { return turn / radius; }
}
