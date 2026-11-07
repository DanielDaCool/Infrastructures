package frc.robot.Path;

import static frc.robot.Path.PathConfig.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;
import java.util.List;

/** Follows a {@link Path}: call {@link #calculate} every {@link TimedRobot#kDefaultPeriod}. */
public final class PathFollower {
    private static final double DT = TimedRobot.kDefaultPeriod; // loop period [s]
    private static final double HEADING_KP = 4.0; // heading error gain [(rad/s) per rad]
    /** Gain on the remaining distance to a stop and on the cross-track error [1/s]. */
    private static final double FINAL_KP = 3.0;
    /** Cross-track distance at which the merge angle is 45 deg [m]. */
    static final double MERGE_DISTANCE = 0.5;
    /** Drivetrain lag the direction lookahead compensates [s]. */
    private static final double LAG_TIME = 0.05;
    /** Least along-track speed toward a stop until within POSITION_TOLERANCE / 2 [m/s]; beats a drive deadband. */
    static final double MIN_SPEED = 0.15;
    /** Measured speed below which a stop counts as reached [m/s]. */
    private static final double STOP_SPEED = 0.1;

    private final Path path;
    private final List<Segment> segments;
    private final List<Path.Stop> stops;
    private final List<Path.Anchor> anchors = new ArrayList<>();
    private int segmentIndex = 0;
    private int stopIndex = 0;
    private int waitLoops = -1; // -1 = not waiting at a stop
    private double progress = 0;
    private double previousSpeed = 0;
    private Translation2d previousVelocity = Translation2d.kZero;
    private boolean finished = false;
    private final double finalHeading;
    private double targetHeading, targetHeadingPerMeter; // set by updateTargetHeading(distance)
    // set by updateProgress and speedAlongPath, valid for the current calculate call only
    private Path.Stop stop;
    private Segment segment;
    private double distanceOnSegment, distanceToStop;

    /** @param startPose robot pose at the start; its heading is used until the first point angle */
    public PathFollower(Path path, Pose2d startPose) {
        this.path = path;
        this.segments = path.segments();
        this.stops = path.stops();
        if (path.anchors().isEmpty() || path.anchors().get(0).distance() > 0)
            anchors.add(new Path.Anchor(0, startPose.getRotation()));
        anchors.addAll(path.anchors());
        finalHeading = anchors.get(anchors.size() - 1).angle().getRadians();
    }

    /**
     * @param pose field pose [m, rad]
     * @param measuredFieldSpeeds measured field-relative speeds [m/s]
     * @return field-relative speed command [m/s, rad/s]
     */
    public ChassisSpeeds calculate(Pose2d pose, ChassisSpeeds measuredFieldSpeeds) {
        if (finished || isBadPose(pose)) return new ChassisSpeeds();
        Translation2d position = pose.getTranslation();
        double heading = pose.getRotation().getRadians();
        updateProgress(position);
        double headingError = MathUtil.angleModulus(targetHeading - heading);

        ChassisSpeeds stopCommand = handleStop(position, heading, headingError, measuredFieldSpeeds);
        if (stopCommand != null) return stopCommand;

        double speed = speedAlongPath();
        Translation2d velocity = velocity(position, speed);
        double turnRate = turnRate(speed, headingError);
        if (!Double.isFinite(velocity.getX()) || !Double.isFinite(velocity.getY()) || !Double.isFinite(turnRate))
            return new ChassisSpeeds();
        previousSpeed = speed;
        previousVelocity = velocity;
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), turnRate);
    }

    /** True if the pose has a NaN or infinite value. */
    private static boolean isBadPose(Pose2d pose) {
        return !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getRotation().getRadians());
    }

    /** Picks the current stop and segment, advances the progress and sets the target heading there. */
    private void updateProgress(Translation2d position) {
        stop = stops.get(stopIndex);
        distanceOnSegment = segments.get(segmentIndex).project(position);
        while (distanceOnSegment >= segments.get(segmentIndex).length() && segmentIndex < segments.size() - 1
                && Math.abs(path.segmentStart(segmentIndex + 1) - stop.distance()) > 1e-9) {
            segmentIndex++;
            distanceOnSegment = segments.get(segmentIndex).project(position);
        }
        segment = segments.get(segmentIndex);
        progress = Math.max(progress,
                path.segmentStart(segmentIndex) + MathUtil.clamp(distanceOnSegment, 0, segment.length()));
        updateTargetHeading(progress);
    }

    /** Finishes at the end, counts the dwell at a stop, and returns the hold command while stopped, else null. */
    private ChassisSpeeds handleStop(Translation2d position, double heading, double headingError,
            ChassisSpeeds measuredFieldSpeeds) {
        double measuredSpeed = Math.hypot(measuredFieldSpeeds.vxMetersPerSecond, measuredFieldSpeeds.vyMetersPerSecond);
        if (!Double.isFinite(measuredSpeed)) measuredSpeed = previousVelocity.getNorm();
        double stopDistance = position.getDistance(path.pointAt(stop.distance()));
        boolean atStop = stopDistance < POSITION_TOLERANCE && measuredSpeed < STOP_SPEED;
        if (stopIndex == stops.size() - 1) {
            if (atStop && Math.abs(MathUtil.angleModulus(finalHeading - heading)) < ANGLE_TOLERANCE) {
                finished = true;
                return new ChassisSpeeds();
            }
        }
        if (stopIndex < stops.size() - 1 && (waitLoops >= 0 || atStop)) {
            if (++waitLoops >= Math.round(stop.dwell() / DT)) {
                stopIndex++;
                waitLoops = -1;
                progress = stop.distance(); // the robot may be up to POSITION_TOLERANCE short of the stop
                segmentIndex = path.indexAt(progress);
            }
        }
        if (waitLoops >= 0 || stopDistance < POSITION_TOLERANCE / 2) {
            previousSpeed = 0;
            previousVelocity = Translation2d.kZero;
            return new ChassisSpeeds(0, 0, MathUtil.clamp(HEADING_KP * headingError, -MAX_OMEGA, MAX_OMEGA));
        }
        return null;
    }

    /** Speed along the path: accelerate, then cap by the arc speeds ahead and the braking to the stop. */
    private double speedAlongPath() {
        distanceToStop = stop.distance() - progress;
        double segmentEnd = path.segmentStart(segmentIndex) + segment.length();
        if (Math.abs(segmentEnd - stop.distance()) < 1e-9)
            distanceToStop = stop.distance() - (path.segmentStart(segmentIndex) + distanceOnSegment); // < 0 past the stop
        double speed = Math.min(previousSpeed + MAX_ACCELERATION * DT, MAX_VELOCITY);
        for (int j = segmentIndex; j < segments.size() && path.segmentStart(j) < stop.distance(); j++) {
            double curvature = Math.abs(segments.get(j).curvature());
            if (curvature == 0) continue;
            double arcSpeedLimit = Math.sqrt(MAX_ACCELERATION / curvature);
            speed = Math.min(speed, Math.sqrt(arcSpeedLimit * arcSpeedLimit
                    + 2 * MAX_ACCELERATION * Math.max(0, path.segmentStart(j) - progress)));
        }
        double approachSpeed = Math.min(Math.sqrt(2 * MAX_ACCELERATION * Math.max(0, distanceToStop)),
                FINAL_KP * distanceToStop);
        approachSpeed = Math.copySign(Math.max(Math.abs(approachSpeed), MIN_SPEED), distanceToStop); // beats a drive deadband
        speed = Math.min(speed, approachSpeed);
        return Math.max(speed, Math.min(0, approachSpeed)); // floor 0, or drive back when past the stop
    }

    /** Field velocity: look-ahead direction merged with the side error, then the acceleration and speed limits. */
    private Translation2d velocity(Translation2d position, double speed) {
        double progressOnSegment = progress - path.segmentStart(segmentIndex);
        Translation2d tangent = segment.tangentAt(progressOnSegment);
        Translation2d normal = new Translation2d(-tangent.getY(), tangent.getX());
        // just before the stop, so the look-ahead stays on the leg arriving at it, not the next one
        Translation2d lookAheadDirection =
                path.tangentAt(Math.min(progress + speed * LAG_TIME, stop.distance() - 1e-9));
        // signed distance from the path, left of the path positive
        double sideError = Path.cross(tangent, position.minus(segment.pointAt(progressOnSegment)));
        double mergeAngle = -Math.atan(sideError / MERGE_DISTANCE);
        // Turn the travel direction toward the path by mergeAngle and keep the speed, so the robot curves
        // onto it. Only close to a stop, where the robot is slow, also pull straight in so it settles.
        double sidewaysSpeed = Math.abs(speed) * Math.sin(mergeAngle);
        if (Math.abs(distanceToStop) < MERGE_DISTANCE)
            sidewaysSpeed = -Math.signum(sideError) * Math.max(Math.abs(sidewaysSpeed), FINAL_KP * Math.abs(sideError));
        sidewaysSpeed = MathUtil.clamp(sidewaysSpeed, -MAX_VELOCITY, MAX_VELOCITY);
        Translation2d velocity =
                lookAheadDirection.times(speed * Math.cos(mergeAngle)).plus(normal.times(sidewaysSpeed));
        Translation2d velocityChange = velocity.minus(previousVelocity);
        double maxVelocityChange = 2 * MAX_ACCELERATION * DT; // vector acceleration limit is twice MAX_ACCELERATION
        if (velocityChange.getNorm() > maxVelocityChange)
            velocity = previousVelocity.plus(velocityChange.times(maxVelocityChange / velocityChange.getNorm()));
        if (velocity.getNorm() > MAX_VELOCITY) velocity = velocity.times(MAX_VELOCITY / velocity.getNorm());
        return velocity;
    }

    /** Turn rate: heading feed-forward along the path plus the heading error correction. */
    private double turnRate(double speed, double headingError) {
        return MathUtil.clamp(targetHeadingPerMeter * speed + HEADING_KP * headingError, -MAX_OMEGA, MAX_OMEGA);
    }

    /** True once the robot is at the end within tolerance, position and heading. */
    public boolean isFinished() { return finished; }

    /** Distance travelled along the path [m]; never decreases. */
    public double progress() { return progress; }

    int segmentIndex() { return segmentIndex; }

    /**
     * Sets targetHeading [rad] and targetHeadingPerMeter [rad/m] at the path distance [m]: linear
     * between anchors along the shortest wrap.
     */
    private void updateTargetHeading(double distance) {
        int i = anchors.size() - 1;
        while (i > 0 && anchors.get(i).distance() > distance) i--;
        Path.Anchor before = anchors.get(i);
        targetHeading = before.angle().getRadians();
        targetHeadingPerMeter = 0;
        if (i == anchors.size() - 1) return;
        Path.Anchor after = anchors.get(i + 1);
        double anchorSpacing = after.distance() - before.distance();
        if (anchorSpacing < 1e-9) return;
        targetHeadingPerMeter = MathUtil.angleModulus(after.angle().getRadians() - targetHeading) / anchorSpacing;
        targetHeading += targetHeadingPerMeter * (distance - before.distance());
    }
}
