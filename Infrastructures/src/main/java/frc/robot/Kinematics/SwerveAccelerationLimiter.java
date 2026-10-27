package frc.robot.Kinematics;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Limits how fast a swerve drive's velocity setpoint can change.
 *
 * <p>Each call to {@link #calculate} returns the velocity closest to the wanted one that the
 * skid, forward and tilt limits allow in the time since the previous call. Velocities are
 * field-relative. Omega is passed through.
 *
 * <p>The limiter only keeps the time of the last call. The caller keeps the last commanded
 * velocity (the speeds actually sent, after desaturation) and passes it every loop. Use one
 * instance per drivetrain and call {@code calculate} once per loop:
 *
 * <pre>{@code
 * wanted = fromRobotRelativeSpeeds(kinematics.desaturate(fromFieldRelativeSpeeds(wanted, heading)), heading);
 * lastFieldSpeeds = limiter.calculate(wanted, lastFieldSpeeds, heading);
 * // send fromFieldRelativeSpeeds(lastFieldSpeeds, heading) with the chassis's own drive method
 * }</pre>
 */
public final class SwerveAccelerationLimiter {
    // Placeholders. Tune on the robot.
    public static final double MAX_FORWARD_ACCEL = 10.0; // forward limit at rest [m/s^2]
    public static final double FREE_SPEED = 5.3; // where the drive motors run out of torque: Kraken X60 6000 rpm (no FOC) / 6.03 * 4" wheel [m/s]
    public static final double MAX_SKID_ACCEL = 8.0; // traction limit [m/s^2]
    public static final double MAX_TILT_ACCEL_FRONT = 12.0; // robot front/back [m/s^2]
    public static final double MAX_TILT_ACCEL_SIDE = 12.0; // robot left/right [m/s^2]
    public static final double MAX_DT = 0.02; // one loop: a slow loop still moves at most one step [s]
    private static final double TOLERANCE = 1e-12; // rounding allowed when checking a limit [m/s]

    private double lastTime = MathSharedStore.getTimestamp();

    /**
     * Returns the next velocity setpoint: the velocity closest to {@code wanted} that the skid,
     * forward and tilt limits allow from {@code lastCommanded} in the time since the previous call.
     *
     * @param wanted field-relative target speeds [m/s, rad/s]
     * @param lastCommanded field-relative speeds sent last loop (after desaturation)
     * @param heading robot heading, CCW positive, the same angle used for field-to-robot conversion
     * @return field-relative speeds to command this loop. Omega is {@code wanted}'s omega (0 if NaN)
     */
    public ChassisSpeeds calculate(ChassisSpeeds wanted, ChassisSpeeds lastCommanded, Rotation2d heading) {
        double now = MathSharedStore.getTimestamp();
        double dt = now - lastTime; // time since the last call [s]
        lastTime = now;
        return calculate(wanted, lastCommanded, heading, dt);
    }

    /** Same as {@link #calculate(ChassisSpeeds, ChassisSpeeds, Rotation2d)} with a given dt [s]. */
    static ChassisSpeeds calculate(ChassisSpeeds wanted, ChassisSpeeds lastCommanded, Rotation2d heading, double dt) {
        return calculate(wanted, lastCommanded, heading, dt,
                MAX_FORWARD_ACCEL, FREE_SPEED, MAX_SKID_ACCEL, MAX_TILT_ACCEL_FRONT, MAX_TILT_ACCEL_SIDE);
    }

    /** The limiter with given limits, so they can be tested with other values than the constants. */
    static ChassisSpeeds calculate(ChassisSpeeds wanted, ChassisSpeeds lastCommanded, Rotation2d heading, double dt,
            double maxForwardAccel, double freeSpeed, double maxSkidAccel, double maxTiltFront, double maxTiltSide) {
        dt = Math.min(dt, MAX_DT);

        // NaN or infinite velocities: stop, instead of returning NaN.
        Translation2d velocity = finiteOrZero(lastCommanded.vxMetersPerSecond, lastCommanded.vyMetersPerSecond);
        Translation2d target = finiteOrZero(wanted.vxMetersPerSecond, wanted.vyMetersPerSecond);
        double omega = Double.isFinite(wanted.omegaRadiansPerSecond) ? wanted.omegaRadiansPerSecond : 0;

        Translation2d wantedChange = target.minus(velocity);
        if (!(dt > 0) || wantedChange.getNorm() == 0) return toSpeeds(velocity, omega); // nothing to do

        // Unknown heading: the tilt axes are unknown, so use the smaller tilt limit on both.
        if (!Double.isFinite(heading.getRadians())) {
            heading = Rotation2d.kZero;
            maxTiltFront = maxTiltSide = Math.min(maxTiltFront, maxTiltSide);
        }

        // Every limit is a set of allowed velocity changes c (field-relative, for this loop):
        // 1. Skid: |c| <= MAX_SKID_ACCEL * dt, a disk around 0.
        // 2. Forward: the speed can grow by at most MAX_FORWARD_ACCEL * (1 - speed / FREE_SPEED) * dt.
        //    |velocity + c| <= speed + maxSpeedUp, a disk around -velocity. Braking and turning
        //    are free here (skid and tilt still limit them). This caps the new speed itself, so a
        //    step to the side can't add speed either (the step is a chord of the turn, not the arc).
        // 3. Tilt: |c along the robot's front| <= MAX_TILT_ACCEL_FRONT * dt, and the same for the
        //    side: a box turned with the robot, 4 half-planes.
        double speed = velocity.getNorm();
        double maxSpeedUp = maxForwardAccel * Math.max(0, 1 - speed / freeSpeed) * dt;
        Translation2d front = new Translation2d(heading.getCos(), heading.getSin());
        Translation2d side = new Translation2d(-heading.getSin(), heading.getCos());
        double maxFront = maxTiltFront * dt, maxSide = maxTiltSide * dt;

        Disk[] disks = {
                new Disk(new Translation2d(), maxSkidAccel * dt),
                new Disk(velocity.unaryMinus(), speed + maxSpeedUp) };
        HalfPlane[] halfPlanes = {
                new HalfPlane(front, maxFront), new HalfPlane(front.unaryMinus(), maxFront),
                new HalfPlane(side, maxSide), new HalfPlane(side.unaryMinus(), maxSide) };

        // All the sets hold c = 0 (no change), so the allowed changes are never empty. The closest
        // allowed change to the wanted one is the wanted change itself, or on the edge of one limit
        // (the wanted change pushed onto it), or where the edges of two limits cross. Try them all
        // and keep the closest one that every limit allows.
        Translation2d best = new Translation2d();
        double bestDistance = wantedChange.getNorm();
        Translation2d[] candidates = candidates(wantedChange, disks, halfPlanes);
        for (Translation2d c : candidates) {
            if (c == null) continue;
            double distance = wantedChange.minus(c).getNorm();
            if (distance < bestDistance && allowed(c, disks, halfPlanes)) {
                best = c;
                bestDistance = distance;
            }
        }

        return toSpeeds(velocity.plus(best), omega);
    }

    /** A disk of allowed changes: |c - center| <= radius. */
    private record Disk(Translation2d center, double radius) {}

    /** A half-plane of allowed changes: c . normal <= limit (normal is a unit vector). */
    private record HalfPlane(Translation2d normal, double limit) {}

    private static boolean allowed(Translation2d c, Disk[] disks, HalfPlane[] halfPlanes) {
        for (Disk d : disks)
            if (c.minus(d.center).getNorm() > d.radius + TOLERANCE) return false;
        for (HalfPlane h : halfPlanes)
            if (c.dot(h.normal) > h.limit + TOLERANCE) return false;
        return true;
    }

    /** The wanted change, its closest point on each edge, and where each two edges cross. */
    private static Translation2d[] candidates(Translation2d wantedChange, Disk[] disks, HalfPlane[] halfPlanes) {
        Translation2d[] out = new Translation2d[1 + disks.length + halfPlanes.length
                + 2 * disks.length * (disks.length - 1) / 2
                + 2 * disks.length * halfPlanes.length
                + halfPlanes.length * (halfPlanes.length - 1) / 2];
        int n = 0;
        out[n++] = wantedChange;
        for (Disk d : disks) {
            Translation2d fromCenter = wantedChange.minus(d.center);
            double size = fromCenter.getNorm();
            out[n++] = size > 0 ? d.center.plus(fromCenter.times(d.radius / size)) : null;
        }
        for (HalfPlane h : halfPlanes) {
            out[n++] = wantedChange.minus(h.normal.times(wantedChange.dot(h.normal) - h.limit));
        }
        for (int i = 0; i < disks.length; i++) {
            for (int j = i + 1; j < disks.length; j++) {
                Translation2d[] p = crossing(disks[i], disks[j]);
                out[n++] = p[0];
                out[n++] = p[1];
            }
        }
        for (Disk d : disks) {
            for (HalfPlane h : halfPlanes) {
                Translation2d[] p = crossing(d, h);
                out[n++] = p[0];
                out[n++] = p[1];
            }
        }
        for (int i = 0; i < halfPlanes.length; i++) {
            for (int j = i + 1; j < halfPlanes.length; j++) {
                out[n++] = crossing(halfPlanes[i], halfPlanes[j]);
            }
        }
        return out;
    }

    /** Where two circles cross (null if they don't). */
    private static Translation2d[] crossing(Disk a, Disk b) {
        Translation2d between = b.center.minus(a.center);
        double d = between.getNorm();
        if (d == 0 || d > a.radius + b.radius || d < Math.abs(a.radius - b.radius)) return new Translation2d[2];
        double along = (a.radius * a.radius - b.radius * b.radius + d * d) / (2 * d);
        double across = Math.sqrt(Math.max(0, a.radius * a.radius - along * along));
        Translation2d unit = between.div(d);
        Translation2d middle = a.center.plus(unit.times(along));
        Translation2d normal = new Translation2d(-unit.getY(), unit.getX());
        return new Translation2d[] { middle.plus(normal.times(across)), middle.minus(normal.times(across)) };
    }

    /** Where a circle crosses a half-plane's edge line (null if it doesn't). */
    private static Translation2d[] crossing(Disk d, HalfPlane h) {
        double distance = h.limit - d.center.dot(h.normal); // from the center to the line, along the normal
        if (Math.abs(distance) > d.radius) return new Translation2d[2];
        double across = Math.sqrt(Math.max(0, d.radius * d.radius - distance * distance));
        Translation2d foot = d.center.plus(h.normal.times(distance));
        Translation2d along = new Translation2d(-h.normal.getY(), h.normal.getX());
        return new Translation2d[] { foot.plus(along.times(across)), foot.minus(along.times(across)) };
    }

    /** Where two half-planes' edge lines cross (null if parallel). */
    private static Translation2d crossing(HalfPlane a, HalfPlane b) {
        double det = a.normal.getX() * b.normal.getY() - a.normal.getY() * b.normal.getX();
        if (Math.abs(det) < 1e-12) return null;
        return new Translation2d(
                (a.limit * b.normal.getY() - b.limit * a.normal.getY()) / det,
                (a.normal.getX() * b.limit - b.normal.getX() * a.limit) / det);
    }

    private static Translation2d finiteOrZero(double x, double y) {
        return Double.isFinite(x + y) ? new Translation2d(x, y) : new Translation2d();
    }

    private static ChassisSpeeds toSpeeds(Translation2d velocity, double omega) {
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
    }
}
