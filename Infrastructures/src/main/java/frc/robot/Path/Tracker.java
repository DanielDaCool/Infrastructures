package frc.robot.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The follower's state machine and the translation and heading laws; the finish test; the
 * follower constants (design doc sections 8.2 to 8.7). Owns the projector, speed law and event
 * schedule of one run. {@link FollowPathCommand} delegates to it. Time is passed in, so the
 * tracker runs the same in a robot loop and in a test.
 */
final class Tracker {

    /** Follower states (8.14). */
    enum State { TRACK, CAPTURE_STOP, ROTATE, DWELL, CAPTURE_END, DONE }

    /** Nominal loop period [s] (8.2.2). */
    static final double DT_NOM = 0.02;
    /** Loop period clamp [s] (8.2.2). */
    static final double DT_MIN = 0.005;
    /** Loop period clamp [s] (8.2.2). */
    static final double DT_MAX = 0.10;
    /** Largest hand-over distance d_h [m] (8.3.7); d_h = min(this, jerkMax*DT_NOM/kP^2). */
    static final double D_H_MAX = 0.05;
    /** Correction clamp c_max [m/s] (8.3.8). */
    static final double C_MAX = 1.0;
    /** Minimum lookahead of the projection window [m] (8.2.3). */
    static final double AHEAD_MIN = 0.15;
    /** Window lookahead in loops (8.2.3). */
    static final int K_AHEAD = 3;
    /** Maximum lookahead of the projection window [m] (8.2.3). */
    static final double AHEAD_MAX = 0.50;
    /** Window look-back eps_back [m] (8.2.4). */
    static final double EPS_BACK = 0.03;
    /** Minimum arc window beyond the arc start, {@code max(this, (pi/2) r_j)} [m] (8.2.3). */
    static final double ARC_WINDOW_MIN = 0.05;
    /** Face-target freeze radius r_face_min [m] (6.9, 8.5.3). */
    static final double R_FACE_MIN = 0.50;
    /** Stop speed [m/s] (7.10.3, 8.6). */
    static final double STOP_SPEED = 0.1;
    /** Stop ROTATE entry and exit angle, 3 deg [rad] (6.7, 8.6). */
    static final double STOP_ANG_TOL = Math.toRadians(3.0);
    /** Path shorter than this starts in CAPTURE_END [m] (8.12). */
    static final double L_SHORT = 0.10;
    /** Pose older than this is stale [s] (8.12). */
    static final double T_STALE = 0.10;
    /** Invalid or frozen pose longer than this gives FAILED_POSE [s] (8.12). */
    static final double POSE_INVALID_LIMIT = 1.0;
    /** Stale-pose speed gate [m/s] (8.12). */
    static final double STALE_POSE_SPEED_GATE = 0.3;
    /** With a NaN speed supplier, stop arrival needs abs(d_go) &lt;= pos_tol for this long [s] (8.12). */
    static final double NAN_SPEED_ARRIVAL_TIME = 0.1;
    /** s within this of a goal counts as at the goal (left limits) [m]. */
    private static final double S_TOL = 1e-9;

    private final Path path;
    private final SegmentTable table;
    private final Projector projector;
    private final SpeedProfile profile;
    private final HeadingProfile heading;
    private final SpeedLaw law;
    private final EventSchedule schedule;
    private final PathFollowerConfig config;
    private final PathConstraints constraints;
    private final double length;
    private final double dH;
    private final double vH;
    private final double thetaEnd;
    private final Translation2d pEnd;
    private final double timeout;

    private State state = State.TRACK;
    private FollowResult result = FollowResult.RUNNING;

    // progress
    private double sStar;
    private double sHi;
    private double sPrev;
    private double sSubStart;
    private double sGoal;
    private double sStop = Double.NaN;

    // control state
    private SpeedLaw.State lawState = new SpeedLaw.State(0.0, 0.0);
    private double vtPrev;
    private double vLimPrev;
    private double omegaPrev;
    private double faceRefPrev;

    // timers [s]
    private double tStart;
    private double tPrev = Double.NaN;
    private double tIn;
    private double tFin = Double.NaN;
    private double tDwellStart = Double.NaN;
    private double tArriveTol = Double.NaN;

    // pose validity (8.12)
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private double lastTheta = Double.NaN;
    private double lastChangeT;
    private double invalidSince = Double.NaN;
    private ChassisSpeeds lastCommand = new ChassisSpeeds();
    private boolean speedInvalidSeen;

    // per-loop values (telemetry, tests)
    private double eN;
    private double eT;
    private double dGo;
    private double eTheta;
    private double vMeasAbs;
    private double omegaFF;

    /**
     * @param path        the active (flipped) path
     * @param profile     speed profile built from the measured start speed
     * @param heading     heading profile
     * @param law         the speed law
     * @param schedule    events of the active path
     * @param config      follower gains and tolerances
     * @param constraints resolved constraints (omegaMax, alphaMax, vMax)
     */
    Tracker(Path path, SpeedProfile profile, HeadingProfile heading, SpeedLaw law,
            EventSchedule schedule, PathFollowerConfig config, PathConstraints constraints) {
        this.path = path;
        this.table = path.table();
        this.projector = new Projector(table);
        this.profile = profile;
        this.heading = heading;
        this.law = law;
        this.schedule = schedule;
        this.config = config;
        this.constraints = constraints;
        this.length = table.length();
        this.dH = law.handOver();
        this.vH = config.kP() * dH;
        this.thetaEnd = heading.endHeading();
        this.pEnd = table.evaluate(length).position();
        this.timeout = config.timeoutFactor() * profile.profileTime() + config.timeoutMargin();
    }

    /**
     * Start speed of the speed law, {@code v0 = max(0, V_meas . T(0))}; 0 for a NaN speed (7.6.4, 8.12).
     *
     * @param path     the active path
     * @param measured measured field speeds, may be null
     * @return v0 [m/s]
     */
    static double startSpeed(Path path, ChassisSpeeds measured) {
        if (!speedValid(measured)) {
            return 0.0;
        }
        PathSample s0 = path.table().evaluate(0.0);
        double v = measured.vxMetersPerSecond * s0.tangent().getCos()
                + measured.vyMetersPerSecond * s0.tangent().getSin();
        return Math.max(0.0, v);
    }

    /** @return true if the pose is not null and x, y and theta are finite */
    static boolean poseValid(Pose2d pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    /** @return true if the speeds are not null and every component is finite */
    static boolean speedValid(ChassisSpeeds v) {
        return v != null && Double.isFinite(v.vxMetersPerSecond) && Double.isFinite(v.vyMetersPerSecond)
                && Double.isFinite(v.omegaRadiansPerSecond);
    }

    /**
     * Resets the run (8.14 initialize): s = 0, (v_cmd, a_cmd) = (v0, 0), latches, state.
     *
     * @param pose     measured pose (valid)
     * @param measured measured field speeds
     * @param nowSec   time [s]
     */
    void start(Pose2d pose, ChassisSpeeds measured, double nowSec) {
        double v0 = startSpeed(path, measured);
        boolean sv = speedValid(measured);
        tStart = nowSec;
        tPrev = Double.NaN;
        sStar = 0.0;
        sHi = 0.0;
        sPrev = 0.0;
        sSubStart = 0.0;
        sStop = Double.NaN;
        sGoal = profile.nextStopS(0.0);
        lawState = new SpeedLaw.State(v0, 0.0);
        vtPrev = v0;
        vLimPrev = v0;
        omegaPrev = sv ? measured.omegaRadiansPerSecond : 0.0;
        tIn = 0.0;
        tFin = Double.NaN;
        tDwellStart = Double.NaN;
        tArriveTol = Double.NaN;
        lastX = pose.getX();
        lastY = pose.getY();
        lastTheta = pose.getRotation().getRadians();
        lastChangeT = nowSec;
        invalidSince = Double.NaN;
        lastCommand = new ChassisSpeeds();
        speedInvalidSeen = !sv;
        schedule.reset();
        result = FollowResult.RUNNING;
        Translation2d target = heading.faceTarget();
        if (target != null) {
            Translation2d pT = target.minus(pose.getTranslation());
            faceRefPrev = pT.getNorm() >= R_FACE_MIN ? Math.atan2(pT.getY(), pT.getX()) : heading.theta(0.0);
        }
        state = length < L_SHORT ? State.CAPTURE_END : State.TRACK;
        eN = 0.0;
        eT = 0.0;
        dGo = sGoal;
        eTheta = 0.0;
        vMeasAbs = sv ? Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) : 0.0;
    }

    /**
     * One execute (8.14): projection, events, state transitions, translation and heading laws.
     *
     * @param pose     measured pose, blue-origin
     * @param measured measured field speeds
     * @param nowSec   time [s]
     * @return the field-relative command, omega CCW
     */
    ChassisSpeeds update(Pose2d pose, ChassisSpeeds measured, double nowSec) {
        double dt = Double.isNaN(tPrev) ? DT_NOM : clamp(nowSec - tPrev, DT_MIN, DT_MAX);
        tPrev = nowSec;
        if (result != FollowResult.RUNNING || state == State.DONE) {
            return new ChassisSpeeds();
        }
        boolean sv = speedValid(measured);
        speedInvalidSeen |= !sv;
        vMeasAbs = sv ? Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) : Double.NaN;

        // 8.12: invalid or stale pose
        if (!checkPose(pose, sv, nowSec)) {
            double bad = nowSec - invalidSince;
            if (bad > POSE_INVALID_LIMIT) {
                result = FollowResult.FAILED_POSE;
                return new ChassisSpeeds();
            }
            return bad < T_STALE ? lastCommand : new ChassisSpeeds();
        }
        Translation2d p = pose.getTranslation();
        double theta = pose.getRotation().getRadians();

        // 8.2.3, 8.2.4: windowed projection (ROTATE and DWELL hold s* = s_c)
        if (state == State.TRACK || state == State.CAPTURE_STOP || state == State.CAPTURE_END) {
            double vHat = sv ? Math.max(vtPrev, vMeasAbs) : vtPrev;
            double ahead = clamp(K_AHEAD * vHat * dt, AHEAD_MIN, AHEAD_MAX);
            double sMin = Math.max(sSubStart, sHi - EPS_BACK);
            double sMax = Math.max(sMin, windowMax(ahead));
            sStar = projector.project(p, sMin, sMax, sPrev).s();
            sHi = Math.max(sHi, sStar);
            sPrev = sStar;
            schedule.update(sStar); // 8.8
        }
        geometry(p);

        // 8.6, 8.7: state machine
        switch (state) {
            case TRACK:
                if (dGo <= dH || (lawState.v() == 0.0 && dGo <= dH + 2.0 * SpeedLaw.STOP_DIST_MARGIN)) {
                    vLimPrev = vtPrev;
                    tArriveTol = Double.NaN;
                    tIn = 0.0;
                    state = atEndGoal() ? State.CAPTURE_END : State.CAPTURE_STOP;
                }
                break;
            case CAPTURE_STOP: {
                boolean posOk = Math.abs(dGo) <= config.posTol();
                boolean arrived;
                if (sv) {
                    arrived = posOk && vMeasAbs <= STOP_SPEED;
                } else {
                    if (!posOk) {
                        tArriveTol = Double.NaN;
                    } else if (Double.isNaN(tArriveTol)) {
                        tArriveTol = nowSec;
                    }
                    arrived = posOk && nowSec - tArriveTol >= NAN_SPEED_ARRIVAL_TIME;
                }
                if (arrived) {
                    sStop = sGoal;
                    sStar = sStop;
                    sHi = sStop;
                    sPrev = sStop;
                    sSubStart = sStop;
                    sGoal = profile.nextStopS(sStop);
                    schedule.fireStopEvents(sStop);
                    if (Math.abs(PathMath.wrap(heading.theta(sStop) - theta)) > STOP_ANG_TOL) {
                        state = State.ROTATE;
                    } else {
                        tDwellStart = nowSec;
                        state = State.DWELL;
                    }
                    geometry(p);
                }
                break;
            }
            case ROTATE:
                if (Math.abs(PathMath.wrap(heading.theta(sStop) - theta)) <= STOP_ANG_TOL) {
                    tDwellStart = nowSec;
                    state = State.DWELL;
                }
                break;
            case DWELL:
                if (nowSec - tDwellStart >= profile.dwellAt(sStop)) {
                    PathSample out = table.evaluate(sStop);
                    double v = sv ? Math.max(0.0, measured.vxMetersPerSecond * out.tangent().getCos()
                            + measured.vyMetersPerSecond * out.tangent().getSin()) : 0.0;
                    lawState = new SpeedLaw.State(v, 0.0);
                    vtPrev = v;
                    state = State.TRACK;
                    geometry(p);
                }
                break;
            case CAPTURE_END: {
                boolean inTol = p.getDistance(pEnd) < config.posTol()
                        && Math.abs(PathMath.wrap(thetaEnd - theta)) < config.angTolRad();
                tIn = inTol ? tIn + dt : 0.0;
                if (tIn >= config.tSettle() && Double.isNaN(tFin)) {
                    tFin = nowSec;
                }
                break;
            }
            default:
                break;
        }

        // 8.3, 8.4: translation
        double vx;
        double vy;
        double vt;
        if (state == State.TRACK) {
            PathSample at = sample(sStar);
            double nx = -at.tangent().getSin();
            double ny = at.tangent().getCos();
            double lead = sStar + vtPrev * config.tauLag();
            double sL = Math.min(lead, sGoal);
            double sV = Math.min(lead, sGoal - dH);
            double c = clamp(-config.kN() * eN, -C_MAX, C_MAX);
            double budget = Math.sqrt(Math.max(0.0, constraints.vMax() * constraints.vMax() - c * c));
            double cap = Math.min(profile.capAt(sV), budget);
            lawState = law.step(lawState, sV, sGoal, cap, dt);
            if (lawState.v() > budget) { // 8.3.9: a jump of c cuts v_t at once and resets the law
                lawState = new SpeedLaw.State(budget, Math.min(lawState.a(), 0.0));
            }
            vt = lawState.v();
            PathSample look = sample(sL);
            vx = vt * look.tangent().getCos() + c * nx;
            vy = vt * look.tangent().getSin() + c * ny;
            vtPrev = vt;
        } else {
            Translation2d goal = goalPoint();
            double[] v = captureLaw(p, goal, dt);
            vx = v[0];
            vy = v[1];
            vt = Math.hypot(vx, vy);
            vtPrev = vt;
        }

        // 8.5: heading
        double ref;
        double ff;
        Translation2d target = heading.faceTarget();
        switch (state) {
            case TRACK:
                if (target != null) {
                    double px = target.getX() - p.getX();
                    double py = target.getY() - p.getY();
                    double d2 = px * px + py * py;
                    if (Math.sqrt(d2) < R_FACE_MIN) {
                        ref = faceRefPrev;
                        ff = 0.0;
                    } else {
                        ref = Math.atan2(py, px);
                        ff = (py * vx - px * vy) / d2;
                    }
                    faceRefPrev = ref;
                } else {
                    ref = heading.theta(sStar);
                    ff = heading.dTheta(sStar) * vt;
                }
                break;
            case CAPTURE_STOP:
                ref = heading.theta(sGoal);
                ff = 0.0;
                break;
            case ROTATE:
            case DWELL:
                ref = heading.theta(sStop);
                ff = 0.0;
                break;
            default:
                ref = thetaEnd;
                ff = 0.0;
                break;
        }
        double omega = headingLaw(ref, ff, theta, dt);
        lastCommand = new ChassisSpeeds(vx, vy, omega);
        return lastCommand;
    }

    /** @return false if the pose is NaN or stale (8.12); sets invalidSince */
    private boolean checkPose(Pose2d pose, boolean sv, double nowSec) {
        if (!poseValid(pose)) {
            if (Double.isNaN(invalidSince)) {
                invalidSince = nowSec;
            }
            return false;
        }
        double x = pose.getX();
        double y = pose.getY();
        double th = pose.getRotation().getRadians();
        if (x == lastX && y == lastY && th == lastTheta) {
            if (nowSec - lastChangeT >= T_STALE && sv && vMeasAbs > STALE_POSE_SPEED_GATE) {
                if (Double.isNaN(invalidSince)) {
                    invalidSince = lastChangeT;
                }
                return false;
            }
        } else {
            lastX = x;
            lastY = y;
            lastTheta = th;
            lastChangeT = nowSec;
        }
        invalidSince = Double.NaN;
        return true;
    }

    /** @return sMax of the window before the floor (8.2.3) */
    private double windowMax(double ahead) {
        double sMax = Math.min(sPrev + ahead, sGoal);
        int kCur = table.segmentIndexAt(sPrev);
        int kNext = kCur;
        for (int k = kCur + 1; k < table.size(); k++) {
            if (table.segment(k).length > 0.0) {
                kNext = k;
                break;
            }
        }
        sMax = Math.min(sMax, table.segment(kNext).sEnd());
        for (int k = kCur; k < table.size(); k++) {
            Segment seg = table.segment(k);
            if (seg.sStart > sMax) {
                break;
            }
            if (seg.type == Segment.Type.ARC && seg.sEnd() >= sPrev) {
                sMax = Math.min(sMax, Math.max(seg.sStart, sPrev) + Math.max(ARC_WINDOW_MIN, 0.5 * Math.PI * seg.r));
            }
        }
        return sMax;
    }

    /** @return the sample at s; the left limit (leg arrived on) at the current goal (8.1) */
    private PathSample sample(double s) {
        return s >= sGoal - S_TOL ? table.evaluateLeft(s) : table.evaluate(s);
    }

    /** Updates e_n, e_t and d_go at s* (8.2.3, 8.3.7). */
    private void geometry(Translation2d p) {
        PathSample at = sample(sStar);
        double tx = at.tangent().getCos();
        double ty = at.tangent().getSin();
        double dx = p.getX() - at.position().getX();
        double dy = p.getY() - at.position().getY();
        eN = -dx * ty + dy * tx;
        eT = dx * tx + dy * ty;
        if (state == State.TRACK) {
            dGo = (sGoal - sStar) - eT;
        } else {
            // the 2D states: d_go = (P_goal - p) . T_in (8.6), equal to the formula above on the last line
            PathSample g = table.evaluateLeft(sGoal);
            dGo = (g.position().getX() - p.getX()) * g.tangent().getCos()
                    + (g.position().getY() - p.getY()) * g.tangent().getSin();
        }
    }

    /** @return true if the current goal is the end of the path */
    private boolean atEndGoal() {
        return sGoal >= length - S_TOL;
    }

    /** @return the point the 2D law holds: P_end, P_c being captured, or P_c of ROTATE / DWELL */
    private Translation2d goalPoint() {
        switch (state) {
            case CAPTURE_STOP:
                return table.evaluateLeft(sGoal).position();
            case ROTATE:
            case DWELL:
                return table.evaluateLeft(sStop).position();
            default:
                return pEnd;
        }
    }

    /** The 2D goal law of CAPTURE (8.3.7e); updates v_lim_prev. @return {vx, vy} */
    private double[] captureLaw(Translation2d p, Translation2d goal, double dt) {
        double dx = goal.getX() - p.getX();
        double dy = goal.getY() - p.getY();
        double d = Math.hypot(dx, dy);
        double vRaw = config.kP() * d;
        double aAccEff = profile.accelLimit(vLimPrev, Math.min(sStar, length));
        if (!(aAccEff > 0.0)) {
            aAccEff = constraints.aAcc();
        }
        double vLim = Math.min(Math.min(sqrtLaw(d, constraints.aDec(), config.kP()), vH), vLimPrev + aAccEff * dt);
        double scale = vRaw > vLim && vRaw > 0.0 ? vLim / vRaw : 1.0;
        double vx = config.kP() * dx * scale;
        double vy = config.kP() * dy * scale;
        vLimPrev = Math.hypot(vx, vy);
        return new double[] {vx, vy};
    }

    /** Feedback, clamps and rate limit of 8.5.2; sets eTheta and omega_prev. @return omega [rad/s] */
    private double headingLaw(double ref, double ff, double theta, double dt) {
        eTheta = PathMath.wrap(ref - theta);
        omegaFF = ff;
        double fb = sqrtLaw(eTheta, constraints.alphaMax(), config.kTheta());
        double raw = clamp(ff + fb, -constraints.omegaMax(), constraints.omegaMax());
        double band = constraints.alphaMax() * dt;
        double omega = clamp(raw, omegaPrev - band, omegaPrev + band);
        omegaPrev = omega;
        return omega;
    }

    /**
     * The sqrt goal law vsqrt(d; a, k) of 8.3.7: linear {@code k d} for abs(d) &lt;= a/k^2, else
     * {@code sqrt(2 a (abs(d) - a/(2 k^2)))}, with the sign of d.
     */
    static double sqrtLaw(double d, double a, double k) {
        double ad = Math.abs(d);
        double v = ad <= a / (k * k) ? k * ad : Math.sqrt(2.0 * a * (ad - a / (2.0 * k * k)));
        return Math.signum(d) * v;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    /**
     * The finish and timeout test of isFinished (8.7.2, 8.7.3).
     *
     * @param nowSec time [s]
     * @return true when the run is over; {@link #result()} says why
     */
    boolean isFinished(double nowSec) {
        if (result != FollowResult.RUNNING) {
            return true;
        }
        if (nowSec - tStart >= timeout) {
            result = FollowResult.FAILED_TIMEOUT;
            return true;
        }
        if (state == State.CAPTURE_END && !Double.isNaN(tFin) && nowSec - tFin >= profile.dwellAt(length)) {
            schedule.fireRemaining();
            result = FollowResult.OK;
            state = State.DONE;
            return true;
        }
        return false;
    }

    /** @return the current result, RUNNING until it ends */
    FollowResult result() {
        return result;
    }

    /** @return the current state */
    State state() {
        return state;
    }

    /** @return progress s* [m] */
    double s() {
        return sStar;
    }

    /** @return highest accepted progress s_hi [m] */
    double sHi() {
        return sHi;
    }

    /** @return the current goal s_goal (next stop or L) [m] */
    double sGoal() {
        return sGoal;
    }

    /** @return cross-track error e_n [m] */
    double eN() {
        return eN;
    }

    /** @return along-track residual e_t [m] */
    double eT() {
        return eT;
    }

    /** @return signed distance to go d_go [m] */
    double dGo() {
        return dGo;
    }

    /** @return the speed law's v_cmd in TRACK, the 2D law's speed otherwise [m/s] */
    double vRef() {
        return vtPrev;
    }

    /** @return the speed law's acceleration a_cmd [m/s^2] */
    double aCmd() {
        return lawState.a();
    }

    /** @return abs(V_meas) of the last loop [m/s], NaN if the speed supplier gave NaN */
    double vMeas() {
        return vMeasAbs;
    }

    /** @return true if the speed supplier gave a non-finite value in this run (alert A3) */
    boolean speedInvalidSeen() {
        return speedInvalidSeen;
    }

    /** @return heading error in degrees */
    double eThetaDeg() {
        return Math.toDegrees(eTheta);
    }

    /** @return omega feed-forward of the last loop [rad/s] */
    double omegaFF() {
        return omegaFF;
    }

    /** @return segment index at s* */
    int segment() {
        return table.segmentIndexAt(sStar);
    }

    /** @return t_timeout = factor profileTime + margin [s] (8.7.3) */
    double timeout() {
        return timeout;
    }

    /** @return the finish heading theta_end [rad] */
    double endHeading() {
        return thetaEnd;
    }

    /**
     * Prepares {@link #holdEnd}: the 2D law restarts from the measured speed.
     *
     * @param measured measured field speeds
     */
    void startHold(ChassisSpeeds measured) {
        boolean sv = speedValid(measured);
        vLimPrev = sv ? Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) : 0.0;
        omegaPrev = sv ? measured.omegaRadiansPerSecond : 0.0;
        tPrev = Double.NaN;
    }

    /**
     * The CAPTURE_END law on P_end with heading theta_end, used by holdEndCommand (8.7.4). An
     * invalid pose gives zero.
     *
     * @param pose     measured pose
     * @param measured measured field speeds (unused but for symmetry with update)
     * @param nowSec   time [s]
     * @return the field-relative command, omega CCW
     */
    ChassisSpeeds holdEnd(Pose2d pose, ChassisSpeeds measured, double nowSec) {
        double dt = Double.isNaN(tPrev) ? DT_NOM : clamp(nowSec - tPrev, DT_MIN, DT_MAX);
        tPrev = nowSec;
        if (!poseValid(pose)) {
            return new ChassisSpeeds();
        }
        double[] v = captureLaw(pose.getTranslation(), pEnd, dt);
        double omega = headingLaw(thetaEnd, 0.0, pose.getRotation().getRadians(), dt);
        return new ChassisSpeeds(v[0], v[1], omega);
    }
}
