package frc.robot.Path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Path.calculator.ArcCalculator;
import frc.robot.Path.calculator.LegCalculator;
import frc.robot.Path.constants.PathConstants;
import frc.robot.Path.segments.ArcSegment;
import frc.robot.Path.segments.LineSegment;
import frc.robot.Path.segments.SegmentBase;
import frc.robot.Path.trapezoid.DemaciaTrapezoid;

public class DemaciaTrajectory {

    private List<SegmentBase> segments;
    private DemaciaTrapezoid trapezoid;
    private ProfiledPIDController headingController;

    private int currentSegmentIndex;
    private boolean isFinishedTrajectory;
    private boolean resetHeadingController;

    public DemaciaTrajectory(List<PathPoint> points) {
        this.segments = new ArrayList<SegmentBase>();
        this.trapezoid = new DemaciaTrapezoid(PathConstants.MAX_VELOCITY, PathConstants.MAX_ACCELERATION);
        this.headingController = new ProfiledPIDController(PathConstants.HEADING_KP, 0, 0,
                new Constraints(PathConstants.MAX_ANGULAR_VELOCITY, PathConstants.MAX_ANGULAR_ACCELERATION));
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.resetHeadingController = true;

        if (points.size() < 2) {
            System.out.println("Not enough points to build a trajectory");
            this.isFinishedTrajectory = true;
            return;
        }

        buildSegments(points);
        limitEndVelocities();

        for (int i = 0; i < segments.size(); i++) {
            System.out.println("segments.get(" + i + "): " + segments.get(i));
        }
    }

    public void startTrajectory() {
        isFinishedTrajectory = segments.isEmpty();
        currentSegmentIndex = 0;
        resetHeadingController = true;
    }

    public boolean getIsFinishedTrajectory() {
        return isFinishedTrajectory;
    }

    private void buildSegments(List<PathPoint> points) {
        int size = points.size();
        Translation2d[] centers = new Translation2d[size]; // the point itself when it has no arc
        double[] signedRadius = new double[size]; // m, positive = left arc, negative = right arc, 0 = no arc
        double[] velocities = new double[size]; // m/s, when passing the point
        Translation2d[] entry = new Translation2d[size]; // where the leg from the previous point ends
        Translation2d[] exit = new Translation2d[size]; // where the leg to the next point starts

        for (int i = 0; i < size; i++) {
            centers[i] = points.get(i).getTranslation();
        }

        // the corner circle of every point
        for (int i = 1; i < size - 1; i++) {
            Translation2d prev = points.get(i - 1).getTranslation();
            Translation2d here = points.get(i).getTranslation();
            Translation2d next = points.get(i + 1).getTranslation();
            double v = points.get(i).getVelocity();

            double turn = Math.abs(MathUtil.angleModulus(
                    next.minus(here).getAngle().getRadians() - here.minus(prev).getAngle().getRadians()));
            double maxR = Math.min(prev.getDistance(here), here.getDistance(next)) * PathConstants.MAX_RADIUS_TO_LEG_RATIO;
            boolean reversal = turn > Math.PI - PathConstants.MIN_TURN_FOR_ARC;

            if (v == 0 || reversal || turn < PathConstants.MIN_TURN_FOR_ARC || maxR < 1e-6) {
                velocities[i] = reversal ? 0 : v;
                continue;
            }

            double r = v * v / PathConstants.MAX_LATERAL_ACCELERATION;
            if (r > maxR) {
                r = maxR;
                v = Math.sqrt(PathConstants.MAX_LATERAL_ACCELERATION * r);
            }
            ArcSegment corner = ArcCalculator.calculateCenter(prev, here, next, r);
            centers[i] = corner.getCenter();
            signedRadius[i] = corner.getIsLeft() ? r : -r;
            velocities[i] = v;
        }

        // every leg is the line that touches both of its corner circles
        // an arc that can't be built is removed and its point becomes a plain point, then the legs are built again
        boolean removedArc = true;
        while (removedArc) {
            removedArc = false;
            for (int i = 0; i < size - 1; i++) {
                List<Translation2d> leg = LegCalculator.tangentPoints(centers[i], signedRadius[i], centers[i + 1], signedRadius[i + 1]);
                if (leg.isEmpty()) {
                    centers[i] = points.get(i).getTranslation();
                    signedRadius[i] = 0;
                    centers[i + 1] = points.get(i + 1).getTranslation();
                    signedRadius[i + 1] = 0;
                    removedArc = true;
                    break;
                }
                exit[i] = leg.get(0);
                entry[i + 1] = leg.get(1);
            }
            if (removedArc) {
                continue;
            }
            for (int i = 1; i < size - 1; i++) {
                if (signedRadius[i] != 0 && !isArcValid(centers[i], signedRadius[i] > 0, entry[i], points.get(i).getTranslation(), exit[i])) {
                    System.out.println("removed the arc of point " + i);
                    centers[i] = points.get(i).getTranslation();
                    signedRadius[i] = 0;
                    removedArc = true;
                }
            }
        }

        for (int i = 1; i < size; i++) {
            Rotation2d heading = points.get(i).getRotation();
            segments.add(new LineSegment(exit[i - 1], new Pose2d(entry[i], heading), velocities[i], PathConstants.MAX_VELOCITY));
            if (signedRadius[i] != 0) {
                segments.add(new ArcSegment(entry[i], new Pose2d(exit[i], heading), centers[i], signedRadius[i] > 0,
                        Math.abs(signedRadius[i]), velocities[i], velocities[i]));
            }
        }
    }

    // the arc must go from the entry through its point to the exit without going almost all the way around
    private boolean isArcValid(Translation2d center, boolean isLeft, Translation2d entry, Translation2d point, Translation2d exit) {
        double entryAngle = entry.minus(center).getAngle().getRadians();
        double pointAngle = point.minus(center).getAngle().getRadians();
        double exitAngle = exit.minus(center).getAngle().getRadians();
        double entryToPoint = MathUtil.inputModulus(isLeft ? pointAngle - entryAngle : entryAngle - pointAngle, 0, 2 * Math.PI);
        double pointToExit = MathUtil.inputModulus(isLeft ? exitAngle - pointAngle : pointAngle - exitAngle, 0, 2 * Math.PI);
        return entryToPoint + pointToExit < PathConstants.MAX_ARC_SWEEP;
    }

    // lower each end velocity so the next segment can still slow down to its own end velocity
    private void limitEndVelocities() {
        for (int i = segments.size() - 2; i >= 0; i--) {
            SegmentBase segment = segments.get(i);
            SegmentBase next = segments.get(i + 1);
            double reachable = Math.sqrt(next.getEndVelocity() * next.getEndVelocity()
                    + 2 * PathConstants.MAX_ACCELERATION * next.getLength());
            segment.setEndVelocity(Math.min(segment.getEndVelocity(), Math.min(next.getMaxVelocity(), reachable)));
        }
    }

    public ChassisSpeeds calculateSpeeds(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        if (isFinishedTrajectory) {
            return new ChassisSpeeds();
        }
        double heading = currentPose.getRotation().getRadians();
        if (resetHeadingController) {
            headingController.reset(heading, currentSpeeds.omegaRadiansPerSecond);
            resetHeadingController = false;
        }

        SegmentBase segment = segments.get(currentSegmentIndex);
        Translation2d position = currentPose.getTranslation();
        double currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        double speed = Math.min(trapezoid.nextVelocity(segment.getDistanceLeft(position), currentVelocity,
                segment.getEndVelocity(), segment.getMaxVelocity()), segment.getMaxVelocity());
        Translation2d velocity = new Translation2d(speed, segment.getDirection(position, currentVelocity));

        double omega = headingController.calculate(heading, segment.getEndPose().getRotation().getRadians())
                + headingController.getSetpoint().velocity;
        ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);

        if (!segment.isFinished(position)) {
            return speeds;
        }
        // a stop point is done only once the robot really stopped
        if (segment.getEndVelocity() == 0 && currentVelocity > PathConstants.MAX_VELOCITY_TO_CONSIDER_STOPPED) {
            return new ChassisSpeeds(0, 0, omega);
        }
        if (currentSegmentIndex < segments.size() - 1) {
            currentSegmentIndex++;
            return speeds;
        }
        if (Math.abs(currentPose.getRotation().minus(segment.getEndPose().getRotation()).getRadians()) < PathConstants.HEADING_TOLERANCE) {
            isFinishedTrajectory = true;
            return new ChassisSpeeds();
        }
        return new ChassisSpeeds(0, 0, omega);
    }
}
