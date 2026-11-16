package frc.robot.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Path.constants.PathConstants;

// rotation = wanted robot heading at this point, velocity 0 = stop exactly here
public class PathPoint extends Pose2d {
    private final double velocity; // m/s

    public PathPoint(Translation2d translation, Rotation2d rotation) {
        this(translation, rotation, PathConstants.MAX_VELOCITY);
    }

    public PathPoint(Translation2d translation, Rotation2d rotation, double velocity) {
        super(translation, rotation);
        this.velocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }
}
