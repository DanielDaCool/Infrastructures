package frc.robot.Path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Path.constants.PathConstants;

public class ArcSegment extends SegmentBase {

    private Translation2d center;
    private boolean isLeft; // left = counter-clockwise
    private double radius; // m
    private double startAngle; // rad, center to start
    private double sweep; // rad, start to end in the turn direction, [0, 2PI)

    public ArcSegment(Translation2d start, Pose2d end, Translation2d center, boolean isLeft, double radius, double endVelocity, double maxVelocity) {
        super(start, end, endVelocity, maxVelocity);
        this.center = center;
        this.isLeft = isLeft;
        this.radius = radius;
        this.startAngle = start.minus(center).getAngle().getRadians();
        this.sweep = MathUtil.inputModulus(turnDirection(end.getTranslation().minus(center).getAngle().getRadians() - startAngle), 0, 2 * Math.PI);
    }

    public Translation2d getCenter() {
        return center;
    }

    public boolean getIsLeft() {
        return isLeft;
    }

    private double turnDirection(double angle) {
        return isLeft ? angle : -angle;
    }

    // angle left to turn from position to the end, positive = still before the end
    private double remainingAngle(Translation2d position) {
        double robotFromStart = turnDirection(position.minus(center).getAngle().getRadians() - startAngle);
        // wrap opposite the arc's middle so arcs over 180 deg are measured correctly
        double travelled = MathUtil.angleModulus(robotFromStart - sweep / 2) + sweep / 2;
        return sweep - travelled;
    }

    @Override
    public Rotation2d getDirection(Translation2d position, double currentVelocity) {
        Translation2d centerToChassis = position.minus(center);
        double tangent = centerToChassis.getAngle().getRadians() + (isLeft ? Math.PI / 2 : -Math.PI / 2);
        double lookAhead = turnDirection((currentVelocity / radius) * 0.02);

        Translation2d toEnd = getEndPose().getTranslation().minus(position);
        double remaining = remainingAngle(position);
        if (toEnd.getNorm() < 1e-6 || remaining <= 0) {
            return new Rotation2d(tangent + lookAhead);
        }
        // a robot on the arc sees the end half the remaining angle inside the tangent
        double error = MathUtil.angleModulus(toEnd.getAngle().getRadians() - (tangent + turnDirection(remaining / 2)));
        // far off the arc: aim straight at the end
        if (Math.abs(error) > Math.PI / 2) {
            return toEnd.getAngle();
        }
        // same as the line: turn twice the error to pull the robot back onto the arc
        return new Rotation2d(tangent + lookAhead + 2 * error);
    }

    @Override
    public double getDistanceLeft(Translation2d position) {
        return Math.max(0, remainingAngle(position)) * radius;
    }

    @Override
    public boolean isFinished(Translation2d position) {
        return remainingAngle(position) <= 0
                || position.getDistance(getEndPose().getTranslation()) < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH;
    }

    @Override
    public double getLength() {
        return radius * sweep;
    }

    @Override
    public String toString() {
        return super.toString() + " Center Circle: " + center + " Radius: " + radius + " Sweep: " + sweep;
    }
}
