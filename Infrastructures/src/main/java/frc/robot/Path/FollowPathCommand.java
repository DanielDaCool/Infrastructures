package frc.robot.Path;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Kinematics.KinematicsConfig;

/**
 * Drives a {@link Path} (design doc section 9.10). A command, not a value: the {@code withX}
 * methods configure this object before it is scheduled and return it.
 *
 * <p>Each loop it sends one field-relative {@link ChassisSpeeds} (blue frame, omega CCW) to the
 * output; it never negates omega, never discretizes and never limits translation (8.11). On a
 * failure it ends and sends zero speeds once (8.7.4).
 */
public class FollowPathCommand extends Command {

    private final Path path;
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> measuredFieldSpeeds;
    private final Consumer<ChassisSpeeds> output;
    private final Subsystem[] requirements;
    private final PathTelemetry telemetry = new PathTelemetry();
    private final Map<String, AlertType> activeAlerts = new LinkedHashMap<>();

    private KinematicsConfig kinematics;
    private PathsConfig pathsConfig;
    private PathFollowerConfig config = PathFollowerConfig.defaults();
    private Consumer<FollowResult> onFinish;
    private DoubleSupplier clock = Timer::getFPGATimestamp;

    private boolean published;
    private Tracker tracker;
    private Path active;
    private FollowResult result = FollowResult.RUNNING;
    private String failReason = "";
    private boolean speedAlertRaised;

    /**
     * @param path                the path (blue frame)
     * @param pose                blue-origin field pose
     * @param measuredFieldSpeeds field-relative measured speeds, omega CCW
     * @param output              receives the field-relative command, omega CCW
     * @param requirements        the drivetrain (at least one, V24)
     */
    public FollowPathCommand(Path path,
                             Supplier<Pose2d> pose,
                             Supplier<ChassisSpeeds> measuredFieldSpeeds,
                             Consumer<ChassisSpeeds> output,
                             Subsystem... requirements) {
        this.path = Objects.requireNonNull(path, Point.nullMessage("FollowPathCommand", "the path"));
        this.pose = Objects.requireNonNull(pose, Point.nullMessage("FollowPathCommand", "the pose supplier"));
        this.measuredFieldSpeeds = Objects.requireNonNull(measuredFieldSpeeds,
                Point.nullMessage("FollowPathCommand", "the measured speed supplier"));
        this.output = Objects.requireNonNull(output, Point.nullMessage("FollowPathCommand", "the output"));
        if (requirements == null || requirements.length == 0) {
            throw new IllegalArgumentException("FollowPathCommand needs the drivetrain as a requirement, for example"
                    + " new FollowPathCommand(path, swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative,"
                    + " swerve). Without it, two commands could drive the robot at the same time.");
        }
        for (int i = 0; i < requirements.length; i++) {
            Objects.requireNonNull(requirements[i], Point.nullMessage("FollowPathCommand", "requirement " + i));
        }
        this.requirements = requirements.clone();
        addRequirements(this.requirements);
        EventSchedule.checkRequirements(path.events(), getRequirements());
        setName("FollowPath(" + path.name() + ")");
    }

    /** @return this; motor curve, wheel cap, limit warning C2 */
    public FollowPathCommand withKinematics(KinematicsConfig config) {
        this.kinematics = Objects.requireNonNull(config, Point.nullMessage("withKinematics", "the config"));
        return this;
    }

    /** @return this; R_mod for the wheel-speed cap (moduleRadius 0 = unknown) */
    public FollowPathCommand withPathsConfig(PathsConfig config) {
        this.pathsConfig = Objects.requireNonNull(config, Point.nullMessage("withPathsConfig", "the config"));
        return this;
    }

    /** @return this; gains, tolerances, timeout, flip (default {@code PathFollowerConfig.defaults()}) */
    public FollowPathCommand withConfig(PathFollowerConfig config) {
        this.config = Objects.requireNonNull(config, Point.nullMessage("withConfig", "the config"));
        return this;
    }

    /** @return this; called once with the result when the command ends */
    public FollowPathCommand withOnFinish(Consumer<FollowResult> callback) {
        this.onFinish = Objects.requireNonNull(callback, Point.nullMessage("withOnFinish", "the callback"));
        return this;
    }

    /** Test hook: the time source [s] (default {@code Timer.getFPGATimestamp}). @return this */
    FollowPathCommand withClock(DoubleSupplier clock) {
        this.clock = Objects.requireNonNull(clock);
        return this;
    }

    @Override
    public void initialize() {
        for (Map.Entry<String, AlertType> e : activeAlerts.entrySet()) {
            PathWarnings.runtimeAlert(e.getKey(), e.getValue(), false); // A1 to A3 cleared at the next initialize
        }
        activeAlerts.clear();
        if (!published) {
            PathLogger.get().putData("Path/" + path.name(), telemetry);
            published = true;
        }
        tracker = null;
        result = FollowResult.RUNNING;
        failReason = "";
        speedAlertRaised = false;
        double now = clock.getAsDouble();

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            raise(prefix() + "alliance unknown; driving the blue path.", AlertType.kWarning);
        }
        active = path.forCurrentAlliance(config);

        Pose2d p = pose.get();
        ChassisSpeeds vm = measuredFieldSpeeds.get();
        if (!Tracker.poseValid(p)) {
            fail(FollowResult.FAILED_POSE, "the pose was invalid or frozen for more than 1 s");
            return;
        }
        checkSpeed(vm);
        Translation2d p0 = active.startPoint();
        double d0 = p.getTranslation().getDistance(p0);
        if (d0 > config.startTol()) {
            fail(FollowResult.FAILED_START_TOO_FAR, String.format(Locale.US,
                    "the robot is %.2f m from the start point (%.2f, %.2f) and must start within %.2f m; check the"
                            + " start pose and the alliance flip", d0, p0.getX(), p0.getY(), config.startTol()));
            return;
        }

        PathConstraints constraints = resolveConstraints();
        HeadingProfile heading = HeadingProfile.build(active, p.getRotation().getRadians());
        double v0 = Tracker.startSpeed(active, vm);
        double moduleRadius = pathsConfig != null ? pathsConfig.moduleRadius() : 0.0;
        SpeedProfile profile = SpeedProfile.build(active, heading, constraints, kinematics, moduleRadius, v0,
                config.kP(), config.posTol());
        if (profile.profileGuardHit()) {
            PathWarnings.startWarning(prefix() + "profile simulation did not reach the goal; the timeout uses the"
                    + " guard time (a bug, please report it).");
        }
        SpeedLaw law = new SpeedLaw(profile);
        EventSchedule schedule = new EventSchedule(active.events(), active.table().stopS());
        tracker = new Tracker(active, profile, heading, law, schedule, config, constraints);
        tracker.start(p, vm, now);
        telemetry.setState(tracker.state().name());
    }

    @Override
    public void execute() {
        if (tracker == null) {
            return;
        }
        double now = clock.getAsDouble();
        Pose2d p = pose.get();
        ChassisSpeeds vm = measuredFieldSpeeds.get();
        checkSpeed(vm);
        ChassisSpeeds cmd = tracker.update(p, vm, now);
        output.accept(cmd);
        FollowResult r = tracker.result();
        String state = r.isFailure() ? r.name() : tracker.state().name();
        telemetry.update(state, tracker.segment(), tracker.s(), tracker.eN(), tracker.vRef(),
                Tracker.speedValid(vm) ? Math.hypot(vm.vxMetersPerSecond, vm.vyMetersPerSecond) : Double.NaN,
                tracker.eThetaDeg());
    }

    @Override
    public boolean isFinished() {
        if (result != FollowResult.RUNNING) {
            return true;
        }
        if (tracker == null) {
            return false;
        }
        if (tracker.isFinished(clock.getAsDouble())) {
            result = tracker.result();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        output.accept(new ChassisSpeeds()); // zero once (8.7.4)
        if (result == FollowResult.RUNNING && tracker != null) {
            result = tracker.result();
        }
        if (interrupted && result == FollowResult.RUNNING) {
            result = FollowResult.INTERRUPTED;
        }
        if (result == FollowResult.FAILED_TIMEOUT) {
            failReason = String.format(Locale.US, "did not finish within %.2f s (%s x the planned time + %s s); the"
                    + " robot was blocked, or the limits are above what the drivetrain can do", tracker.timeout(),
                    trim(config.timeoutFactor()), trim(config.timeoutMargin()));
        } else if (result == FollowResult.FAILED_POSE && failReason.isEmpty()) {
            failReason = "the pose was invalid or frozen for more than 1 s";
        }
        if (result.isFailure()) {
            raise(prefix() + result.name() + ": " + failReason + ".", AlertType.kError);
        }
        telemetry.setState(result == FollowResult.OK ? Tracker.State.DONE.name() : result.name());
        if (onFinish != null) {
            onFinish.accept(result);
        }
    }

    /** @return why the command ended, RUNNING until then */
    public FollowResult getResult() {
        return result;
    }

    /**
     * A fresh command holding P_end with the CAPTURE_END law until interrupted (8.7.4), for
     * {@code follow.andThen(follow.holdEndCommand())}. It needs a run that got past the start check;
     * otherwise (FAILED_START_TOO_FAR, FAILED_POSE at start) it sends zero speeds.
     *
     * @return the hold command, requiring the same subsystems
     */
    public Command holdEndCommand() {
        return new FunctionalCommand(
                () -> {
                    if (tracker != null) {
                        tracker.startHold(measuredFieldSpeeds.get());
                    }
                },
                () -> output.accept(tracker != null
                        ? tracker.holdEnd(pose.get(), measuredFieldSpeeds.get(), clock.getAsDouble())
                        : new ChassisSpeeds()),
                interrupted -> output.accept(new ChassisSpeeds()),
                () -> false,
                requirements);
    }

    /** Internal (tests): the tracker of the current run, or null before initialize or after a start failure. */
    Tracker tracker() {
        return tracker;
    }

    /** Internal (tests): the telemetry Sendable. */
    PathTelemetry telemetry() {
        return telemetry;
    }

    // ---------------------------------------------------------------- helpers

    private String prefix() {
        return "Path \"" + path.name() + "\": ";
    }

    private void fail(FollowResult r, String reason) {
        result = r;
        failReason = reason;
        telemetry.setState(r.name());
    }

    private void raise(String text, AlertType type) {
        PathWarnings.runtimeAlert(text, type, true);
        activeAlerts.put(text, type);
    }

    /** Raises A3 once per run if the measured speed is not a number (8.12). */
    private void checkSpeed(ChassisSpeeds vm) {
        if (!speedAlertRaised && !Tracker.speedValid(vm)) {
            speedAlertRaised = true;
            raise(prefix() + "the measured speed is not a number; check the speed supplier.", AlertType.kWarning);
        }
    }

    /** Constraints rule of 9.10, with warnings C1 and C2 (9.12.3). */
    private PathConstraints resolveConstraints() {
        PathsConfig paths = pathsConfig != null ? pathsConfig : PathsConfig.defaults();
        PathConstraints c;
        if (active.constraints().isPresent()) {
            c = active.constraints().get();
        } else if (kinematics != null) {
            c = PathConstraints.fromConfig(kinematics, paths);
        } else {
            PathWarnings.startWarning(prefix() + "no PathConstraints and no KinematicsConfig: using the defaults of a"
                    + " new KinematicsConfig() (placeholders). Pass PathConstraints.fromConfig(yourConfig).");
            c = PathConstraints.fromConfig(new KinematicsConfig());
        }
        String clip = "the limiter will clip it and the robot will fall behind the profile";
        if (kinematics != null) {
            PathConstraints derived = PathConstraints.fromConfig(kinematics, paths);
            warnAbove("vMax", c.vMax(), "the value from the KinematicsConfig", derived.vMax(), clip);
            warnAbove("aAcc", c.aAcc(), "the value from the KinematicsConfig", derived.aAcc(), clip);
            warnAbove("aDec", c.aDec(), "the value from the KinematicsConfig", derived.aDec(), clip);
            warnAbove("aLat", c.aLat(), "the value from the KinematicsConfig", derived.aLat(), clip);
            warnAbove("vMax", c.vMax(), "maxAllowedModuleVelocity", kinematics.maxAllowedModuleVelocity, clip);
        }
        List<Point> points = active.points();
        for (int i = 0; i < points.size(); i++) {
            OptionalDouble mv = points.get(i).maxVelocity();
            if (mv.isPresent()) {
                warnAbove("point " + i + " withVelocity", mv.getAsDouble(), "vMax", c.vMax(), "it has no effect");
            }
        }
        warnAbove("jerkMax", c.jerkMax(), "max(aAcc, aDec)/(5 dt)",
                PathWarnings.jerkWarningThreshold(c.aAcc(), c.aDec(), Tracker.DT_NOM),
                "the jerk limit has no effect at 50 Hz, and the robot can enter an arc still braking");
        return c;
    }

    private void warnAbove(String what, double v, String limit, double lim, String effect) {
        if (v > lim + 1e-9) {
            PathWarnings.startWarning(prefix() + what + " " + trim(v) + " is above " + limit + " (" + trim(lim)
                    + "); " + effect + ".");
        }
    }

    private static String trim(double v) {
        return String.format(Locale.US, "%.3f", v).replaceAll("0+$", "").replaceAll("\\.$", "");
    }
}
