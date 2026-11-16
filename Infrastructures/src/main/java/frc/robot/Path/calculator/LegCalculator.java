package frc.robot.Path.calculator;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LegCalculator {

    // returns the points where the straight leg leaves circle 1 and touches circle 2
    // signed radius: positive = left (counter-clockwise) arc, negative = right arc, 0 = a plain point
    // returns an empty list when there is no such leg (the circles are too close)
    public static List<Translation2d> tangentPoints(Translation2d center1, double signedRadius1, Translation2d center2, double signedRadius2){
        List<Translation2d> points = new ArrayList<>();
        Translation2d distance = center2.minus(center1);

        if (distance.getNorm() < 1e-9) {
            points.add(center1);
            points.add(center2);
            return points;
        }
        if (Math.abs(signedRadius2 - signedRadius1) >= distance.getNorm()) {
            return points;
        }

        //the angle between the line connecting the centers and the leg
        double alpha = Math.asin((signedRadius2 - signedRadius1) / distance.getNorm());

        //unit vector pointing to the left of the leg
        Translation2d left = new Translation2d(1, new Rotation2d(distance.getAngle().getRadians() - alpha + Math.PI / 2));

        points.add(center1.minus(left.times(signedRadius1)));
        points.add(center2.minus(left.times(signedRadius2)));
        return points;
    }
}
