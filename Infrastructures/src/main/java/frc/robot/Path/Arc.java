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
     * @param radius radius [m], > 0
     * @param isLeftTurn true for a left (CCW) turn, false for a right (CW) turn
     * @param startAngle polar angle of the start point around the center [rad]
     * @param sweep turned angle [rad], > 0
     */
    public Arc(Translation2d center, double radius, boolean isLeftTurn, double startAngle, double sweep) {
        if (!(radius > 0) || !(sweep > 0)) throw new IllegalArgumentException("Bad arc");
        this.center = center;
        this.radius = radius;
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
    public Translation2d pointAt(double distance) {
        double angle = startAngle + turn * distance / radius;
        return center.plus(new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle)));
    }

    @Override
    public Translation2d tangentAt(double distance) {
        double angle = startAngle + turn * distance / radius;
        return new Translation2d(-turn * Math.sin(angle), turn * Math.cos(angle));
    }

    @Override
    public double project(Translation2d point) {
        Translation2d offset = point.minus(center);
        double angle = Math.atan2(offset.getY(), offset.getX());
        return radius * (MathUtil.angleModulus(turn * (angle - startAngle) - sweep / 2) + sweep / 2);
    }

    @Override
    public double curvature() { return turn / radius; }
}
