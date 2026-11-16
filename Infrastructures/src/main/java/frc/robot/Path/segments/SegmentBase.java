package frc.robot.Path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SegmentBase {

    private Translation2d startTranslation;
    private Pose2d endPose; // end position + wanted robot heading at the end
    private double endVelocity;
    private double maxVelocity;

    public SegmentBase(Translation2d startTranslation, Pose2d endPose, double endVelocity, double maxVelocity) {
        this.startTranslation = startTranslation;
        this.endPose = endPose;
        this.endVelocity = endVelocity;
        this.maxVelocity = maxVelocity;
    }

    // field direction to drive now
    public abstract Rotation2d getDirection(Translation2d position, double currentVelocity);

    public abstract double getDistanceLeft(Translation2d position);

    public abstract boolean isFinished(Translation2d position);

    public abstract double getLength();

    public Pose2d getEndPose() {
        return endPose;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public void setEndVelocity(double endVelocity) {
        this.endVelocity = endVelocity;
    }

    @Override
    public String toString() {
        return "Start point: " + startTranslation + " Finish Point: " + endPose
                + " End velocity: " + endVelocity + " Max velocity: " + maxVelocity;
    }
}
