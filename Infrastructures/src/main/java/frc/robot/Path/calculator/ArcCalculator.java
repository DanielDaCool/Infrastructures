package frc.robot.Path.calculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Path.segments.ArcSegment;

public class ArcCalculator {

    // the returned arc only carries the center and turn direction
    public static ArcSegment calculateCenter(Translation2d from, Translation2d midPoint,Translation2d to, double radius){
        double MidPointToFromAngle = from.minus(midPoint).getAngle().getRadians();
        double MidToPointToAngle = to.minus(midPoint).getAngle().getRadians();
        double vecAngle = (MidPointToFromAngle + MidToPointToAngle) / 2;
        if (Math.abs(MidPointToFromAngle - MidToPointToAngle) > Math.PI) vecAngle += Math.PI;
        double angleDiff = MathUtil.angleModulus(MidToPointToAngle - MidPointToFromAngle);
        boolean isLeftTurn = angleDiff < 0;
        Translation2d center = midPoint.plus(new Translation2d(radius * Math.cos(vecAngle), radius * Math.sin(vecAngle)));
        System.out.println("angleDiff: " + angleDiff + " from: " + from + " midPoint: "  + midPoint + " to: " + to + " vecAngle: " + vecAngle + " center: " + center + " isLeftTurn: " + isLeftTurn + " MidPointToFromAngle: " + MidPointToFromAngle + " MidToPointToAngle: " + MidToPointToAngle);
        return new ArcSegment(from, new Pose2d(to, Rotation2d.kZero), center, isLeftTurn, radius, 0, 0);
    }
}
