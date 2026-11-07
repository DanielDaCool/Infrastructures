package frc.robot.Path;

import static frc.robot.Path.PathConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;

/** Follows a {@link Path}: call {@link #calculate} every {@link PathConstants#DT}. */
public final class PathFollower {
    private final Path path;
    private final List<Segment> segments;
    private final List<Path.Stop> stops;
    private final List<Path.Anchor> anchors = new ArrayList<>();
    private int k = 0;
    private int stopIndex = 0;
    private int waitLoops = -1; // -1 = not waiting at a stop
    private double s = 0;
    private double vPrev = 0;
    private Translation2d vCmdPrev = Translation2d.kZero;
    private boolean finished = false;
    private final double thetaEnd;
    private double thetaRef, dThetaRef; // set by heading(s)

    /** @param startPose robot pose at the start; its heading is used until the first point angle */
    public PathFollower(Path path, Pose2d startPose) {
        this.path = path;
        this.segments = path.segments();
        this.stops = path.stops();
        if (path.anchors().isEmpty() || path.anchors().get(0).s() > 0)
            anchors.add(new Path.Anchor(0, startPose.getRotation()));
        anchors.addAll(path.anchors());
        thetaEnd = anchors.get(anchors.size() - 1).angle().getRadians();
    }

    /**
     * @param pose field pose [m, rad]
     * @param measuredFieldSpeeds measured field-relative speeds [m/s]
     * @return field-relative speed command [m/s, rad/s]
     */
    public ChassisSpeeds calculate(Pose2d pose, ChassisSpeeds measuredFieldSpeeds) {
        Translation2d p = pose.getTranslation();
        double theta = pose.getRotation().getRadians();
        if (finished || !Double.isFinite(p.getX()) || !Double.isFinite(p.getY()) || !Double.isFinite(theta))
            return new ChassisSpeeds();
        Path.Stop stop = stops.get(stopIndex);
        double t = segments.get(k).project(p);
        while (t >= segments.get(k).length() && k < segments.size() - 1
                && Math.abs(path.segmentStart(k + 1) - stop.s()) > 1e-9) {
            k++;
            t = segments.get(k).project(p);
        }
        Segment seg = segments.get(k);
        s = Math.max(s, path.segmentStart(k) + MathUtil.clamp(t, 0, seg.length()));
        heading(s);
        double headingError = MathUtil.angleModulus(thetaRef - theta);

        boolean atStop = p.getDistance(path.pointAt(stop.s())) < POSITION_TOLERANCE
                && Math.hypot(measuredFieldSpeeds.vxMetersPerSecond, measuredFieldSpeeds.vyMetersPerSecond) < STOP_SPEED;
        if (stopIndex == stops.size() - 1) {
            if (atStop && Math.abs(MathUtil.angleModulus(thetaEnd - theta)) < ANGLE_TOLERANCE) {
                finished = true;
                return new ChassisSpeeds();
            }
        } else if (waitLoops >= 0 || atStop) {
            if (++waitLoops >= Math.round(stop.dwell() / DT)) {
                stopIndex++;
                waitLoops = -1;
                s = stop.s(); // the robot may be up to POSITION_TOLERANCE short of the stop
                k = path.indexAt(s);
            }
            vPrev = 0;
            vCmdPrev = Translation2d.kZero;
            return new ChassisSpeeds(0, 0, MathUtil.clamp(HEADING_KP * headingError, -MAX_OMEGA, MAX_OMEGA));
        }

        double toStop = stop.s() - s;
        double segEnd = path.segmentStart(k) + seg.length();
        if (Math.abs(segEnd - stop.s()) < 1e-9) toStop = stop.s() - (path.segmentStart(k) + t); // < 0 past the stop
        double v = Math.min(vPrev + MAX_ACCELERATION * DT, MAX_VELOCITY);
        for (int j = k; j < segments.size() && path.segmentStart(j) < stop.s(); j++) {
            double curvature = Math.abs(segments.get(j).curvature());
            if (curvature == 0) continue;
            double cap = Math.sqrt(MAX_LATERAL_ACCELERATION / curvature);
            v = Math.min(v, Math.sqrt(cap * cap + 2 * MAX_DECELERATION * Math.max(0, path.segmentStart(j) - s)));
        }
        v = Math.min(v, Math.min(Math.sqrt(2 * MAX_DECELERATION * Math.max(0, toStop)), FINAL_KP * toStop));
        v = Math.max(v, Math.min(0, FINAL_KP * toStop)); // floor 0, or drive back when past the stop

        double local = s - path.segmentStart(k);
        Translation2d tangent = seg.tangentAt(local);
        Translation2d normal = new Translation2d(-tangent.getY(), tangent.getX());
        Translation2d look = path.tangentAt(Math.min(s + v * LAG_TIME, stop.s()));
        double eN = Path.cross(tangent, p.minus(seg.pointAt(local)));
        double psi = -Math.atan(eN / MERGE_DISTANCE);
        double vN = -Math.signum(eN) * Math.max(Math.abs(v * Math.sin(psi)), FINAL_KP * Math.abs(eN));
        vN = MathUtil.clamp(vN, -MAX_VELOCITY, MAX_VELOCITY);
        Translation2d cmd = look.times(v * Math.cos(psi)).plus(normal.times(vN));
        Translation2d dv = cmd.minus(vCmdPrev);
        double maxDv = MAX_VECTOR_ACCELERATION * DT;
        if (dv.getNorm() > maxDv) cmd = vCmdPrev.plus(dv.times(maxDv / dv.getNorm()));

        double omega = MathUtil.clamp(dThetaRef * v + HEADING_KP * headingError, -MAX_OMEGA, MAX_OMEGA);
        if (!Double.isFinite(cmd.getX()) || !Double.isFinite(cmd.getY()) || !Double.isFinite(omega))
            return new ChassisSpeeds();
        vPrev = v;
        vCmdPrev = cmd;
        return new ChassisSpeeds(cmd.getX(), cmd.getY(), omega);
    }

    /** True once the robot is at the end within tolerance, position and heading. */
    public boolean isFinished() { return finished; }

    /** Distance travelled along the path [m]; never decreases. */
    public double progress() { return s; }

    int segmentIndex() { return k; }

    /** Sets thetaRef [rad] and dThetaRef [rad/m]: linear between anchors along the shortest wrap. */
    private void heading(double sAt) {
        int i = anchors.size() - 1;
        while (i > 0 && anchors.get(i).s() > sAt) i--;
        Path.Anchor a = anchors.get(i);
        thetaRef = a.angle().getRadians();
        dThetaRef = 0;
        if (i == anchors.size() - 1) return;
        Path.Anchor b = anchors.get(i + 1);
        double ds = b.s() - a.s();
        if (ds < 1e-9) return;
        dThetaRef = MathUtil.angleModulus(b.angle().getRadians() - thetaRef) / ds;
        thetaRef += dThetaRef * (sAt - a.s());
    }
}
