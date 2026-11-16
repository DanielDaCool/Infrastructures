package frc.robot.Path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Path.constants.PathConstants;

public class LineSegment extends SegmentBase {
    private Translation2d startToEndVector;

    public LineSegment(Translation2d startPoint, Pose2d endPoint, double endVelocity, double maxVelocity) {
        super(startPoint, endPoint, endVelocity, maxVelocity);
        this.startToEndVector = endPoint.getTranslation().minus(startPoint);
    }

    @Override
    public Rotation2d getDirection(Translation2d position, double currentVelocity) {
        Translation2d toEnd = getEndPose().getTranslation().minus(position);
        if (toEnd.getNorm() < 1e-6) {
            return startToEndVector.getAngle();
        }
        // passed the end: aim straight at it
        if (Math.abs(MathUtil.angleModulus(toEnd.getAngle().getRadians() - startToEndVector.getAngle().getRadians())) > Math.PI / 2) {
            return toEnd.getAngle();
        }
        // mirror the robot's direction around the line to pull it back onto the line
        return new Rotation2d(2 * toEnd.getAngle().getRadians() - startToEndVector.getAngle().getRadians());
    }

    @Override
    public double getDistanceLeft(Translation2d position) {
        return getEndPose().getTranslation().minus(position).getNorm();
    }

    @Override
    public boolean isFinished(Translation2d position) {
        Translation2d toEnd = getEndPose().getTranslation().minus(position);
        double distance = toEnd.getNorm();
        if (getEndVelocity() == 0) {
            return distance < PathConstants.MAX_POSITION_THRESHOLD_FINAL_POINT;
        }
        boolean passed = distance > 1e-6 && Math.abs(MathUtil.angleModulus(
                toEnd.getAngle().getRadians() - startToEndVector.getAngle().getRadians())) > PathConstants.MAX_ANGLE_TO_PASS;
        return distance < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH || passed;
    }

    @Override
    public double getLength() {
        return startToEndVector.getNorm();
    }
}
