package frc.robot.Path;

import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Path.PathGeometry.PathSample;

/**
 * Drives a {@link Path} (design doc sections 8, 9.10). A command, not a value: the {@code withX}
 * methods configure this object before it is scheduled and return it.
 *
 * <p>Each loop it sends one field-relative {@link ChassisSpeeds} (blue frame, omega CCW) to the
 * output; it never negates omega, never discretizes and never limits translation (8.11). On a
 * failure it ends, sends zero speeds once and prints the reason to the Driver Station (8.7.4).
 *
 * <p>To act part way along the path, use {@link #progress()} in a trigger, for example
 * {@code new Trigger(() -> follow.progress() > 0.7).onTrue(intake.runCommand())}. The command is
 * also a {@code Sendable}: {@code SmartDashboard.putData("Path", follow)} (or any logger that takes
 * a Sendable) shows state, segment, s, eN, vRef, vMeas and eThetaDeg.
 */
public class FollowPathCommand extends Command {

    /** Why the command ended (9.10). */
    public enum Result {
        /** Still running (or not started). */
        RUNNING,
        /** Position and angle in tolerance for t_settle. */
        OK,
        /** t_timeout passed; zero speed sent once. */
        FAILED_TIMEOUT,
        /** More than startTol from P0 at start. */
        FAILED_START_TOO_FAR,
        /** Pose NaN or infinite for more than 1 s (or at the start). */
        FAILED_POSE,
        /** Cancelled by the scheduler or the driver. */
        INTERRUPTED;

        /** @return true if this is {@link #OK} */
        public boolean isSuccess() {
            return this == OK;
        }

        /** @return true for the FAILED_* results */
        public boolean isFailure() {
            return this == FAILED_TIMEOUT || this == FAILED_START_TOO_FAR || this == FAILED_POSE;
        }
    }

    /** Follower states (8.14). */
    enum State { TRACK, CAPTURE_STOP, ROTATE, DWELL, CAPTURE_END, DONE }

    /** Nominal loop period [s] (8.2.2). */
    static final double DT_NOM = 0.02;
    /** Loop period clamp [s] (8.2.2). */
    static final double DT_MIN = 0.005;
    /** Loop period clamp [s] (8.2.2). */
    static final double DT_MAX = 0.10;
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
    /** Stop speed [m/s] (7.10.3, 8.6). */
    static final double STOP_SPEED = 0.1;
    /** Stop ROTATE entry and exit angle, 3 deg [rad] (6.7, 8.6). */
    static final double STOP_ANG_TOL = Math.toRadians(3.0);
    /** Path shorter than this starts in CAPTURE_END [m] (8.12). */
    static final double L_SHORT = 0.10;
    /** An invalid pose repeats the last command for this long, then sends zero [s] (8.12). */
    static final double T_STALE = 0.10;
    /** Invalid pose longer than this gives FAILED_POSE [s] (8.12). */
    static final double POSE_INVALID_LIMIT = 1.0;
    /** s within this of a goal counts as at the goal (left limits) [m]. */
    private static final double S_TOL = 1e-9;

    private static final String POSE_REASON = "the robot pose was NaN or infinite for more than 1 s; check"
            + " that getPose() returns a real pose and that odometry keeps updating";

    // ---------------------------------------------------------------- configuration
    private final Path path;
    private final PathDrivetrain drivetrain;
    private PathFollowerConfig userConfig; // null: PathFollowerConfig.defaults(), resolved in initialize
    private Consumer<Result> onFinish;
    private DoubleSupplier clock = Timer::getFPGATimestamp;

    // ---------------------------------------------------------------- run
    private PathFollowerConfig config;
    private PathConstraints constraints;
    private PathGeometry table;
    private SpeedProfile profile;          // null until a run got past the start checks
    private SpeedProfile.Heading heading;
    private double length;
    private double dH;
    private double vH;
    private double thetaEnd;
    private Translation2d pEnd;
    private double timeout;
    private State state = State.TRACK;
    private Result result = Result.RUNNING;
    private String failReason = "";
    private boolean speedWarned;

    // progress
    private double sStar;
    private double sHi;
    private double sPrev;
    private double sSubStart;
    private double sGoal;
    private double sStop = Double.NaN;

    // control state
    private double vCmd;                   // speed law command v_cmd (8.3.7a)
    private double vtPrev;
    private double vLimPrev;
    private double omegaPrev;

    // timers [s]
    private double tStart;
    private double tPrev = Double.NaN;
    private double tIn;
    private double tFin = Double.NaN;
    private double tDwellStart = Double.NaN;
    private double invalidSince = Double.NaN;
    private ChassisSpeeds lastCommand = new ChassisSpeeds();

    // per-loop values (telemetry, tests)
    private double eN;
    private double dGo;
    private double eTheta;
    private double vMeasAbs;

    /**
     * @param drivetrain the drivetrain: supplies the pose and speeds, receives the commands, is the
     *                   command's requirement and gives the robot limits used when the path has none
     * @param path       the path (blue frame); flipped at start on the red alliance
     */
    public FollowPathCommand(PathDrivetrain drivetrain, Path path) {
        this.drivetrain = Objects.requireNonNull(drivetrain, Point.nullMessage("FollowPathCommand", "the drivetrain",
                "your drivetrain subsystem, which implements PathDrivetrain (for example new FollowPathCommand(swerve, path))"));
        this.path = Objects.requireNonNull(path, Point.nullMessage("FollowPathCommand", "the path", "a Path (for example new Path(new Point(1, 2), new Point(3, 4)))"));
        addRequirements(drivetrain);
        setName("FollowPath(" + path.name() + ")");
    }

    /** @return this; gains, tolerances, timeout, flip (default {@code PathFollowerConfig.defaults()}) */
    public FollowPathCommand withConfig(PathFollowerConfig config) {
        this.userConfig = Objects.requireNonNull(config, Point.nullMessage("withConfig", "the config", "a PathFollowerConfig (for example PathFollowerConfig.defaults())"));
        return this;
    }

    /** @return this; called once with the result when the command ends */
    public FollowPathCommand withOnFinish(Consumer<Result> callback) {
        this.onFinish = Objects.requireNonNull(callback, Point.nullMessage("withOnFinish", "the callback", "a callback that takes the Result (for example result -> System.out.println(result))"));
        return this;
    }

    /** Test hook: the time source [s] (default {@code Timer.getFPGATimestamp}). @return this */
    FollowPathCommand withClock(DoubleSupplier clock) {
        this.clock = Objects.requireNonNull(clock, Point.nullMessage("withClock", "the clock",
                "a DoubleSupplier of seconds (for example () -> 0.0)"));
        return this;
    }

    // ================================================================ command

    /**
     * Starts a run (8.14): reads the alliance once and flips the path on red, checks the start,
     * builds the heading and speed profiles. A single invalid pose here ends the command at once
     * with FAILED_POSE.
     */
    @Override
    public void initialize() {
        double now = clock.getAsDouble();
        profile = null;
        result = Result.RUNNING;
        failReason = "";
        speedWarned = false;
        state = State.TRACK;
        sStar = 0.0;
        sHi = 0.0;
        config = userConfig != null ? userConfig : PathFollowerConfig.defaults();

        Optional<Alliance> alliance = DriverStation.getAlliance(); // read once (8.10)
        if (alliance.isEmpty()) {
            warn(prefix() + "alliance unknown; driving the blue path. For a red run, connect the Driver Station"
                    + " (or set the alliance in the simulator) before the command starts.");
        }
        Path active = alliance.isPresent() && alliance.get() == Alliance.Red ? path.flipped(config) : path;
        table = active.geometry();
        length = table.length();

        Pose2d p = drivetrain.getPose();
        ChassisSpeeds vm = speedsOrZero(drivetrain.getFieldSpeeds());
        if (!poseValid(p)) {
            fail(Result.FAILED_POSE, POSE_REASON);
            return;
        }
        Translation2d p0 = active.startPoint();
        double d0 = p.getTranslation().getDistance(p0);
        if (d0 > config.startTol()) {
            fail(Result.FAILED_START_TOO_FAR, String.format(Locale.US,
                    "the robot is %.2f m from the start point (%.2f, %.2f) and must start within %.2f m (startTol);"
                            + " put the robot on the start point, check setStartPose and the alliance flip, or raise the"
                            + " tolerance, for example PathFollowerConfig.defaults().withStartTol(1.0)", d0, p0.getX(), p0.getY(), config.startTol()));
            return;
        }

        constraints = active.constraints().orElseGet(() ->
                PathConstraints.fromConfig(drivetrain.getKinematicsConfig(), drivetrain.getPathsConfig()));
        heading = SpeedProfile.Heading.build(active, p.getRotation().getRadians());
        double v0 = alongT0(vm);
        profile = SpeedProfile.build(active, heading, constraints, v0, config.kP(), config.posTol());
        dH = profile.handOver();
        vH = config.kP() * dH;
        thetaEnd = heading.endHeading();
        pEnd = table.evaluate(length).position();
        timeout = config.timeoutFactor() * profile.profileTime() + config.timeoutMargin();

        // reset the run (8.14): s = 0, v_cmd = v0
        tStart = now;
        tPrev = Double.NaN;
        sHi = 0.0;
        sPrev = 0.0;
        sSubStart = 0.0;
        sStop = Double.NaN;
        sGoal = profile.nextStopS(0.0);
        vCmd = v0;
        vtPrev = v0;
        vLimPrev = v0;
        omegaPrev = vm.omegaRadiansPerSecond;
        tIn = 0.0;
        tFin = Double.NaN;
        tDwellStart = Double.NaN;
        invalidSince = Double.NaN;
        lastCommand = new ChassisSpeeds();
        state = length < L_SHORT ? State.CAPTURE_END : State.TRACK;
        if (profile.dwellAt(0.0) > 0.0) { // stop(d) on P_0: hold P_0 for d before the start (8.6)
            sStop = 0.0;
            tDwellStart = now;
            state = State.DWELL;
        }
        eN = 0.0;
        dGo = sGoal;
        eTheta = 0.0;
        vMeasAbs = Math.hypot(vm.vxMetersPerSecond, vm.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        if (profile == null || result != Result.RUNNING) {
            return;
        }
        drivetrain.driveFieldRelative(update(drivetrain.getPose(), drivetrain.getFieldSpeeds(), clock.getAsDouble()));
    }

    /** The finish and timeout test (8.7.2, 8.7.3). */
    @Override
    public boolean isFinished() {
        if (result != Result.RUNNING) {
            return true;
        }
        if (profile == null) {
            return false;
        }
        double now = clock.getAsDouble();
        if (now - tStart >= timeout) {
            result = Result.FAILED_TIMEOUT;
            failReason = String.format(Locale.US, "did not finish within %.2f s (%s x the planned time + %s s); the"
                    + " robot was blocked, or the limits are above what the drivetrain can do; free the robot, or lower"
                    + " the limits, for example constraints.withVMax(2.0)", timeout,
                    trim(config.timeoutFactor()), trim(config.timeoutMargin()));
            return true;
        }
        if (state == State.CAPTURE_END && !Double.isNaN(tFin) && now - tFin >= profile.dwellAt(length)) {
            result = Result.OK;
            state = State.DONE;
            sHi = length;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveFieldRelative(new ChassisSpeeds()); // zero once (8.7.4)
        if (interrupted && result == Result.RUNNING) {
            result = Result.INTERRUPTED;
        }
        if (result.isFailure()) {
            warn(prefix() + result.name() + ": " + failReason + ".");
        }
        if (onFinish != null) {
            onFinish.accept(result);
        }
    }

    /** @return why the command ended, RUNNING until then */
    public Result getResult() {
        return result;
    }

    /**
     * @return progress along the path from 0 at the start to 1 at the finish: the highest s* reached
     *     divided by L. It never decreases during a run, so a trigger on it fires once per run.
     */
    public double progress() {
        return profile == null ? 0.0 : Math.min(1.0, sHi / length);
    }

    /** @return the highest distance along the path s* reached in this run [m]; never decreases during a run */
    public double distanceTraveled() {
        return sHi;
    }

    /** Adds the read-only keys state, segment, s, eN, vRef, vMeas, eThetaDeg (8.13) to the Command keys. */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("state", () -> result.isFailure() || result == Result.INTERRUPTED ? result.name()
                : profile == null ? "IDLE" : state.name(), null);
        builder.addIntegerProperty("segment", () -> table == null ? 0 : table.segmentIndexAt(sStar), null);
        builder.addDoubleProperty("s", () -> sStar, null);
        builder.addDoubleProperty("eN", () -> eN, null);
        builder.addDoubleProperty("vRef", () -> vtPrev, null);
        builder.addDoubleProperty("vMeas", () -> vMeasAbs, null);
        builder.addDoubleProperty("eThetaDeg", () -> Math.toDegrees(eTheta), null);
    }

    // ================================================================ tracker (8.2 to 8.7)

    /**
     * One loop (8.14): projection, state transitions, translation and heading laws.
     *
     * @param pose     measured pose, blue-origin
     * @param measured measured field speeds; NaN or infinite counts as 0
     * @param nowSec   time [s]
     * @return the field-relative command, omega CCW
     */
    private ChassisSpeeds update(Pose2d pose, ChassisSpeeds measured, double nowSec) {
        double dt = Double.isNaN(tPrev) ? DT_NOM : clamp(nowSec - tPrev, DT_MIN, DT_MAX);
        tPrev = nowSec;
        if (state == State.DONE) {
            return new ChassisSpeeds();
        }
        ChassisSpeeds vm = speedsOrZero(measured);
        vMeasAbs = Math.hypot(vm.vxMetersPerSecond, vm.vyMetersPerSecond);

        // 8.12: invalid pose: repeat the last command briefly, then zero, then FAILED_POSE
        if (!poseValid(pose)) {
            if (Double.isNaN(invalidSince)) {
                invalidSince = nowSec;
            }
            double bad = nowSec - invalidSince;
            if (bad > POSE_INVALID_LIMIT) {
                fail(Result.FAILED_POSE, POSE_REASON);
                return new ChassisSpeeds();
            }
            return bad < T_STALE ? lastCommand : new ChassisSpeeds();
        }
        invalidSince = Double.NaN;
        Translation2d p = pose.getTranslation();
        double theta = pose.getRotation().getRadians();

        // 8.2.3, 8.2.4: windowed projection (ROTATE and DWELL hold s* = s_c)
        if (state == State.TRACK || state == State.CAPTURE_STOP || state == State.CAPTURE_END) {
            double ahead = clamp(K_AHEAD * Math.max(vtPrev, vMeasAbs) * dt, AHEAD_MIN, AHEAD_MAX);
            double sMin = Math.max(sSubStart, sHi - EPS_BACK);
            double sMax = Math.max(sMin, windowMax(ahead));
            sStar = table.project(p, sMin, sMax, sPrev).s();
            sHi = Math.max(sHi, sStar);
            sPrev = sStar;
        }
        geometry(p);

        // 8.6, 8.7: state machine; ROTATE and DWELL are tested in the loop they are entered, so a zero dwell costs no loop
        switch (state) {
            case TRACK:
                if (dGo <= dH || (vCmd == 0.0 && dGo <= dH + SpeedProfile.REST_HAND_OVER_MARGIN)) {
                    vLimPrev = vtPrev;
                    tIn = 0.0;
                    state = sGoal >= length - S_TOL ? State.CAPTURE_END : State.CAPTURE_STOP;
                }
                break;
            case CAPTURE_STOP:
                if (Math.abs(dGo) <= config.posTol() && vMeasAbs <= STOP_SPEED) {
                    sStop = sGoal;
                    sStar = sStop;
                    sHi = sStop;
                    sPrev = sStop;
                    sSubStart = sStop;
                    sGoal = profile.nextStopS(sStop);
                    if (Math.abs(PathGeometry.wrap(heading.theta(sStop) - theta)) > STOP_ANG_TOL) {
                        state = State.ROTATE;
                    } else {
                        tDwellStart = nowSec;
                        state = State.DWELL;
                    }
                    geometry(p);
                }
                break;
            case CAPTURE_END: {
                boolean inTol = p.getDistance(pEnd) < config.posTol()
                        && Math.abs(PathGeometry.wrap(thetaEnd - theta)) < config.angTolRad();
                tIn = inTol ? tIn + dt : 0.0;
                if (tIn >= PathFollowerConfig.T_SETTLE && Double.isNaN(tFin)) {
                    tFin = nowSec;
                }
                break;
            }
            default:
                break;
        }
        if (state == State.ROTATE && Math.abs(PathGeometry.wrap(heading.theta(sStop) - theta)) <= STOP_ANG_TOL) {
            tDwellStart = nowSec;
            state = State.DWELL;
        }
        if (state == State.DWELL && nowSec - tDwellStart >= profile.dwellAt(sStop)) {
            PathSample out = table.evaluate(sStop);
            double v = Math.max(0.0, vm.vxMetersPerSecond * out.tangent().getCos()
                    + vm.vyMetersPerSecond * out.tangent().getSin());
            vCmd = v;
            vtPrev = v;
            state = length < L_SHORT ? State.CAPTURE_END : State.TRACK;
            geometry(p);
        }

        // 8.3, 8.4: translation
        double vx;
        double vy;
        double vt;
        if (state == State.TRACK) {
            PathSample at = sample(sStar);
            double lead = sStar + vtPrev * PathFollowerConfig.TAU_LAG;
            double sV = Math.min(lead, sGoal - dH);
            double c = clamp(-config.kN() * eN, -C_MAX, C_MAX);
            double budget = Math.sqrt(Math.max(0.0, constraints.vMax() * constraints.vMax() - c * c));
            vCmd = profile.speed(vCmd, sStar, sV, sGoal, budget, dt); // 8.3.9: a jump of c cuts v_t at once
            vt = vCmd;
            PathSample look = sample(Math.min(lead, sGoal));
            vx = vt * look.tangent().getCos() - c * at.tangent().getSin();
            vy = vt * look.tangent().getSin() + c * at.tangent().getCos();
        } else {
            Translation2d goal = switch (state) {
                case CAPTURE_STOP -> table.evaluateLeft(sGoal).position();
                case ROTATE, DWELL -> table.evaluateLeft(sStop).position();
                default -> pEnd;
            };
            double[] v = captureLaw(p, goal, dt);
            vx = v[0];
            vy = v[1];
            vt = Math.hypot(vx, vy);
        }
        vtPrev = vt;

        // 8.5: heading
        double ref = switch (state) {
            case TRACK -> heading.theta(sStar);
            case CAPTURE_STOP -> heading.theta(sGoal);
            case ROTATE, DWELL -> heading.theta(sStop);
            default -> thetaEnd;
        };
        double ff = state == State.TRACK ? heading.dTheta(sStar) * vt : 0.0;
        lastCommand = new ChassisSpeeds(vx, vy, headingLaw(ref, ff, theta, dt));
        return lastCommand;
    }

    /** @return sMax of the window before the floor (8.2.3) */
    private double windowMax(double ahead) {
        double sMax = Math.min(sPrev + ahead, sGoal);
        int kCur = table.segmentIndexAt(sPrev);
        int kNext = kCur;
        for (int k = kCur + 1; k < table.size(); k++) {
            if (table.segment(k).length() > 0.0) {
                kNext = k;
                break;
            }
        }
        sMax = Math.min(sMax, table.segment(kNext).sEnd());
        for (int k = kCur; k < table.size(); k++) {
            PathGeometry.Segment seg = table.segment(k);
            if (seg.sStart() > sMax) {
                break;
            }
            if (seg.type() == PathGeometry.Segment.Type.ARC && seg.sEnd() >= sPrev) {
                sMax = Math.min(sMax, Math.max(seg.sStart(), sPrev) + Math.max(ARC_WINDOW_MIN, 0.5 * Math.PI * seg.r()));
            }
        }
        return sMax;
    }

    /** @return the sample at s; the left limit (leg arrived on) at the current goal (8.1) */
    private PathSample sample(double s) {
        return s >= sGoal - S_TOL ? table.evaluateLeft(s) : table.evaluate(s);
    }

    /** Updates e_n and d_go at s* (8.2.3, 8.3.7). */
    private void geometry(Translation2d p) {
        PathSample at = sample(sStar);
        double tx = at.tangent().getCos();
        double ty = at.tangent().getSin();
        double dx = p.getX() - at.position().getX();
        double dy = p.getY() - at.position().getY();
        eN = -dx * ty + dy * tx;
        if (state == State.TRACK) {
            dGo = (sGoal - sStar) - (dx * tx + dy * ty);
        } else {
            // the 2D states: d_go = (P_goal - p) . T_in (8.6), equal to the formula above on the last line
            PathSample g = table.evaluateLeft(sGoal);
            dGo = (g.position().getX() - p.getX()) * g.tangent().getCos()
                    + (g.position().getY() - p.getY()) * g.tangent().getSin();
        }
    }

    /** The 2D goal law of CAPTURE (8.3.7e); updates v_lim_prev. @return {vx, vy} */
    private double[] captureLaw(Translation2d p, Translation2d goal, double dt) {
        double dx = goal.getX() - p.getX();
        double dy = goal.getY() - p.getY();
        double vRaw = config.kP() * Math.hypot(dx, dy);
        double aAccEff = constraints.aAcc();
        double vLim = Math.min(Math.min(sqrtLaw(Math.hypot(dx, dy), constraints.aDec(), config.kP()), vH),
                vLimPrev + aAccEff * dt);
        double scale = vRaw > vLim && vRaw > 0.0 ? vLim / vRaw : 1.0;
        double vx = config.kP() * dx * scale;
        double vy = config.kP() * dy * scale;
        vLimPrev = Math.hypot(vx, vy);
        return new double[] {vx, vy};
    }

    /** Feedback, clamps and rate limit of 8.5.2; sets eTheta and omega_prev. @return omega [rad/s] */
    private double headingLaw(double ref, double ff, double theta, double dt) {
        eTheta = PathGeometry.wrap(ref - theta);
        double fb = sqrtLaw(eTheta, constraints.alphaMax(), PathFollowerConfig.K_THETA);
        double raw = clamp(ff + fb, -constraints.omegaMax(), constraints.omegaMax());
        double band = constraints.alphaMax() * dt;
        omegaPrev = clamp(raw, omegaPrev - band, omegaPrev + band);
        return omegaPrev;
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

    // ================================================================ helpers

    /** @return v0 = max(0, V_meas . T(0)) (7.6.4) */
    private double alongT0(ChassisSpeeds vm) {
        PathSample s0 = table.evaluate(0.0);
        return Math.max(0.0, vm.vxMetersPerSecond * s0.tangent().getCos() + vm.vyMetersPerSecond * s0.tangent().getSin());
    }

    /** @return the speeds, or zero (with a warning once per run) if null, NaN or infinite (8.12) */
    private ChassisSpeeds speedsOrZero(ChassisSpeeds v) {
        if (v != null && Double.isFinite(v.vxMetersPerSecond) && Double.isFinite(v.vyMetersPerSecond)
                && Double.isFinite(v.omegaRadiansPerSecond)) {
            return v;
        }
        if (!speedWarned) {
            speedWarned = true;
            warn(prefix() + "the measured speed is NaN or infinite; check getFieldSpeeds() of your drivetrain (PathDrivetrain)"
                    + " and that the gyro and encoders are connected.");
        }
        return new ChassisSpeeds();
    }

    /** @return true if the pose is not null and x, y and theta are finite */
    private static boolean poseValid(Pose2d pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private String prefix() {
        return "Path \"" + path.name() + "\": ";
    }

    private void fail(Result r, String reason) {
        result = r;
        failReason = reason;
    }

    private void warn(String text) {
        DriverStation.reportWarning(text, false);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static String trim(double v) {
        return String.format(Locale.US, "%.3f", v).replaceAll("0+$", "").replaceAll("\\.$", "");
    }

    // ---------------------------------------------------------------- internal (tests)

    /** @return the current follower state */
    State state() {
        return state;
    }

    /** @return cross-track error e_n [m] */
    double eN() {
        return eN;
    }

    /** @return heading error in degrees */
    double eThetaDeg() {
        return Math.toDegrees(eTheta);
    }

    /** @return t_timeout = factor profileTime + margin [s] (8.7.3) */
    double timeout() {
        return timeout;
    }
}
