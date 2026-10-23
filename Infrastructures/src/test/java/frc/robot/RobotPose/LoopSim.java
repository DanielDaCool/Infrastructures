package frc.robot.RobotPose;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.OptionalInt;
import java.util.Random;
import java.util.function.DoubleFunction;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BuiltInAccelerometerSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.robot.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.robot.RobotPose.Vision.VisionSource;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * A simulated robot for end-to-end tests of {@link RobotPose}: a truth path, the odometry it
 * produces (gyro + 4 swerve modules, 50 Hz or 250 Hz), and fake devices behind the real vision
 * sources (a MegaTag2 and a 2D Limelight through NetworkTables, a Quest through a fake
 * {@link QuestNav}). {@link #step()} runs one 20 ms robot loop.
 */
final class LoopSim {
    static final double LOOP_SECONDS = 0.02;
    static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3) };

    /** Robot-relative commanded velocity (vx, vy m/s, omega rad/s) at a time since the start. */
    interface Path {
        Twist2d velocityAt(double seconds);
    }

    /** 0-1 s speed up to 2 m/s, 1-3 s curve, 3-4 s stop, 4-5 s spin at 4 rad/s, 5-7 s strafe while turning, then stop. */
    static final Path DRIVE_CURVE_SPIN = t -> {
        double vx = 0;
        double vy = 0;
        double omega = 0;
        if (t < 1) {
            vx = 2 * t;
        } else if (t < 3) {
            vx = 2;
            omega = 0.8 * ramp(t, 1, 1.3);
        } else if (t < 4) {
            vx = 2 * (4 - t);
            omega = 0.8 * (1 - ramp(t, 3, 3.3));
        } else if (t < 5) {
            omega = 4 * ramp(t, 4, 4.2) * (1 - ramp(t, 4.8, 5));
        } else if (t < 7) {
            vy = 1.5 * ramp(t, 5, 5.3) * (1 - ramp(t, 6.7, 7));
            omega = 0.5 * ramp(t, 5, 5.3) * (1 - ramp(t, 6.7, 7));
        }
        return new Twist2d(vx, vy, omega);
    };

    /** Speeds up to 2 m/s in 0.5 s, then drives straight. */
    static final Path STRAIGHT = t -> new Twist2d(2 * Math.min(1, t / 0.5), 0, 0);

    /** Stands still. */
    static final Path STILL = t -> new Twist2d();

    /**
     * A shove at {@code at}: the robot jumps to {@code speed} and slows down with a 0.25 s time
     * constant (moves speed * 0.25 m), the wheels locked.
     */
    static DoubleFunction<Translation2d> shove(double at, Translation2d speed) {
        return t -> t < at ? Translation2d.kZero : speed.times(Math.exp(-(t - at) / 0.25));
    }

    /** Carried by {@code distance} between two times, smoothly (1 - cos speed profile), wheels not turning. */
    static DoubleFunction<Translation2d> carry(double from, double to, Translation2d distance) {
        return t -> t < from || t > to ? Translation2d.kZero
                : distance.div(to - from).times(1 - Math.cos(2 * Math.PI * (t - from) / (to - from)));
    }

    static double ramp(double t, double from, double to) {
        return Math.max(0, Math.min(1, (t - from) / (to - from)));
    }

    final Path path;
    final double startTime;
    /** The wheels report this much more distance than driven (worn wheels, carpet). */
    double wheelScale = 1.02;
    /** 5 samples per loop stamped at their own time (like PhoenixOdometryThread), else 1 at the loop time. */
    boolean highRateOdometry = true;
    /** Field velocity the wheels don't feel (pushed, carried). */
    DoubleFunction<Translation2d> externalVelocity = t -> Translation2d.kZero;
    /** The next odometry sample gets a NaN module distance. */
    boolean nanNextSample = false;

    Pose2d truth;
    private Translation2d fieldVelocity = Translation2d.kZero;
    private final NavigableMap<Double, Pose2d> truthHistory = new TreeMap<>();
    private final double[] moduleDistances = new double[4];
    private final Rotation2d[] moduleAngles = { Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero };
    /** Raw gyro = truth heading + this (the gyro's own zero). */
    private double gyroOffset = 0.7;
    private final List<OdometryData> pendingSamples = new ArrayList<>();
    final List<Rotation2d> poseResetGyroWrites = new ArrayList<>();
    final List<Rotation2d> setYawWrites = new ArrayList<>();
    private final BuiltInAccelerometerSim accelerometer = new BuiltInAccelerometerSim();
    private final List<Translation2d> recentAccelerations = new ArrayList<>();
    final List<FakeLimelight3d> megaTag2s = new ArrayList<>();
    final List<FakeLimelight2d> limelight2ds = new ArrayList<>();
    final List<FakeQuestNav> quests = new ArrayList<>();
    final Random random = new Random(1);

    /** Per loop after the warm-up: (time, position error). */
    final List<double[]> errors = new ArrayList<>();

    LoopSim(Path path, Pose2d start) {
        this.path = path;
        this.truth = start;
        this.startTime = Timer.getFPGATimestamp();
        truthHistory.put(startTime, truth);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        accelerometer.setX(0);
        accelerometer.setY(0);
        accelerometer.setZ(1);
    }

    double time() {
        return Timer.getFPGATimestamp() - startTime;
    }

    Rotation2d gyroAngle() {
        return new Rotation2d(truth.getRotation().getRadians() + gyroOffset);
    }

    /** Writes a new yaw to the gyro: from now on it reads {@code angle} for the current heading. */
    void writeGyro(Rotation2d angle) {
        gyroOffset = angle.getRadians() - truth.getRotation().getRadians();
    }

    /** What RobotPose's odometry supplier returns: every sample since the last call. */
    List<OdometryData> takeSamples() {
        List<OdometryData> samples = new ArrayList<>(pendingSamples);
        pendingSamples.clear();
        return samples;
    }

    private SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(moduleDistances[i], moduleAngles[i]);
        }
        if (nanNextSample) {
            positions[1] = new SwerveModulePosition(Double.NaN, moduleAngles[1]);
            nanNextSample = false;
        }
        return positions;
    }

    private void addSample() {
        pendingSamples.add(new OdometryData(Timer.getFPGATimestamp(), gyroAngle(), modulePositions()));
    }

    /** Moves the truth, the wheels and the clock by 1 ms. */
    private void stepOneMillisecond() {
        double dt = 0.001;
        Twist2d velocity = path.velocityAt(time() + dt / 2);
        Translation2d previousVelocity = fieldVelocity;
        Pose2d before = truth;
        truth = truth.exp(new Twist2d(velocity.dx * dt, velocity.dy * dt, velocity.dtheta * dt));
        truth = new Pose2d(truth.getTranslation().plus(externalVelocity.apply(time() + dt / 2).times(dt)),
                truth.getRotation());
        fieldVelocity = truth.getTranslation().minus(before.getTranslation()).div(dt);
        Translation2d acceleration = fieldVelocity.minus(previousVelocity).div(dt);
        recentAccelerations.add(acceleration.rotateBy(truth.getRotation().unaryMinus()));
        if (recentAccelerations.size() > 20) {
            recentAccelerations.remove(0);
        }
        for (int i = 0; i < 4; i++) {
            Translation2d location = MODULE_LOCATIONS[i];
            Translation2d moduleVelocity = new Translation2d(velocity.dx - velocity.dtheta * location.getY(),
                    velocity.dy + velocity.dtheta * location.getX());
            double speed = moduleVelocity.getNorm();
            if (speed > 1e-6) {
                moduleAngles[i] = moduleVelocity.getAngle();
            }
            moduleDistances[i] += speed * dt * wheelScale;
        }
        SimHooks.stepTiming(dt);
        truthHistory.put(Timer.getFPGATimestamp(), truth);
        truthHistory.headMap(Timer.getFPGATimestamp() - 2, true).clear();
        for (FakeLimelight3d camera : megaTag2s) {
            camera.update();
        }
        for (FakeLimelight2d camera : limelight2ds) {
            camera.update();
        }
        for (FakeQuestNav quest : quests) {
            quest.update();
        }
    }

    /** One 20 ms robot loop: odometry samples, devices, then {@code RobotPose.periodic()}. */
    void step() {
        for (int ms = 1; ms <= 20; ms++) {
            stepOneMillisecond();
            if (highRateOdometry && ms % 4 == 0) {
                addSample();
            }
        }
        if (!highRateOdometry) {
            addSample();
        }
        // The roboRIO accelerometer (label up, X front), averaged over the last 20 ms, in g.
        Translation2d mean = Translation2d.kZero;
        for (Translation2d a : recentAccelerations) {
            mean = mean.plus(a);
        }
        mean = mean.div(Math.max(1, recentAccelerations.size())).div(9.81);
        // The roboRIO accelerometer reads up to 8 g.
        mean = new Translation2d(Math.max(-8, Math.min(8, mean.getX())), Math.max(-8, Math.min(8, mean.getY())));
        accelerometer.setX(mean.getX());
        accelerometer.setY(mean.getY());
        RobotPose.getInstance().periodic();
        Pose2d estimate = RobotPose.getInstance().getEstimatedPose();
        errors.add(new double[] { time(), estimate.getTranslation().getDistance(truth.getTranslation()),
            Math.abs(estimate.getRotation().minus(truth.getRotation()).getRadians()) });
    }

    void run(double seconds) {
        int loops = (int) Math.round(seconds / LOOP_SECONDS);
        for (int i = 0; i < loops; i++) {
            step();
        }
    }

    Pose2d truthAt(double fpgaSeconds) {
        var before = truthHistory.floorEntry(fpgaSeconds);
        return before == null ? truthHistory.firstEntry().getValue() : before.getValue();
    }

    /** {rms, max} position error (m) of the loops between two times since the start. */
    double[] error(double fromSeconds, double toSeconds) {
        double sum = 0;
        double max = 0;
        int count = 0;
        for (double[] e : errors) {
            if (e[0] >= fromSeconds && e[0] <= toSeconds) {
                sum += e[1] * e[1];
                max = Math.max(max, e[1]);
                count++;
            }
        }
        return new double[] { Math.sqrt(sum / Math.max(1, count)), max };
    }

    /** Max heading error (rad) between two times since the start. */
    double headingError(double fromSeconds, double toSeconds) {
        double max = 0;
        for (double[] e : errors) {
            if (e[0] >= fromSeconds && e[0] <= toSeconds) {
                max = Math.max(max, e[2]);
            }
        }
        return max;
    }

    /**
     * A MegaTag2 Limelight: captures at 30 Hz, solves the robot position with the heading it was
     * last sent (through {@code robot_orientation_set}), publishes {@code botpose_orb_wpiblue}
     * after its latency.
     */
    final class FakeLimelight3d {
        final NetworkTable table;
        double periodSeconds = 1.0 / 30;
        double latencySeconds = 0.035;
        double noiseMeters = 0.02;
        /** Added to the heading the Limelight uses (a wrong camera yaw). */
        double headingErrorRadians = 0;
        /** Center of the tags it sees. */
        Translation2d tags = new Translation2d(11.9, 4.03);
        int tagCount = 2;
        double heartbeat = 0;
        private double nextCapture;
        private final List<double[]> inFlight = new ArrayList<>();

        FakeLimelight3d(String name) {
            table = NetworkTableInstance.getDefault().getTable("limelight-" + name);
            nextCapture = Timer.getFPGATimestamp();
            megaTag2s.add(this);
        }

        void update() {
            double now = Timer.getFPGATimestamp();
            if (now >= nextCapture) {
                nextCapture += periodSeconds;
                double[] sent = table.getEntry("robot_orientation_set").getDoubleArray(new double[0]);
                if (sent.length > 0) {
                    double yawUsed = Math.toRadians(sent[0]) + headingErrorRadians;
                    double delta = yawUsed - truth.getRotation().getRadians();
                    Translation2d measured = tags.minus(tags.minus(truth.getTranslation()).rotateBy(new Rotation2d(delta)))
                            .plus(new Translation2d(random.nextGaussian() * noiseMeters, random.nextGaussian() * noiseMeters));
                    inFlight.add(new double[] { now, measured.getX(), measured.getY(), Math.toDegrees(yawUsed),
                        tags.getDistance(truth.getTranslation()) });
                }
            }
            while (!inFlight.isEmpty() && inFlight.get(0)[0] + latencySeconds <= now) {
                double[] frame = inFlight.remove(0);
                heartbeat++;
                table.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[] {
                    frame[1], frame[2], 0, 0, 0, frame[3], (now - frame[0]) * 1000, tagCount, 0.5, frame[4], 0.2 });
                table.getEntry("tv").setDouble(1);
                table.getEntry("hb").setDouble(heartbeat);
            }
        }
    }

    /**
     * A 2D Limelight looking at one tag: publishes {@code t2d} (tx, ty, tid, latencies) for the
     * truth pose after its latency, while the tag is in its field of view.
     */
    final class FakeLimelight2d {
        final NetworkTable table;
        final Transform3d offset;
        final int tagId;
        final Translation3d tag;
        double latencySeconds = 0.03;
        double periodSeconds = 1.0 / 30;
        double noiseDegrees = 0.05;
        int published = 0;
        private double nextCapture;
        private final List<double[]> inFlight = new ArrayList<>();

        /** @param offset Robot to camera; pitch is positive = tilted up (the 2D convention). */
        FakeLimelight2d(String name, Transform3d offset, int tagId) {
            table = NetworkTableInstance.getDefault().getTable("limelight-" + name);
            this.offset = offset;
            this.tagId = tagId;
            this.tag = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTagPose(tagId).get().getTranslation();
            nextCapture = Timer.getFPGATimestamp();
            limelight2ds.add(this);
        }

        void update() {
            double now = Timer.getFPGATimestamp();
            if (now >= nextCapture) {
                nextCapture += periodSeconds;
                // Camera to tag, robot relative and level, then without the camera yaw.
                Translation2d camera = truth.getTranslation().plus(offset.getTranslation().toTranslation2d().rotateBy(truth.getRotation()));
                Translation2d level = tag.toTranslation2d().minus(camera).rotateBy(truth.getRotation().unaryMinus())
                        .rotateBy(new Rotation2d(-offset.getRotation().getZ()));
                double up = tag.getZ() - offset.getZ();
                double pitch = offset.getRotation().getY();
                double forwardCamera = level.getX() * Math.cos(pitch) + up * Math.sin(pitch);
                double upCamera = -level.getX() * Math.sin(pitch) + up * Math.cos(pitch);
                double tx = Math.toDegrees(-Math.atan2(level.getY(), forwardCamera)) + random.nextGaussian() * noiseDegrees;
                double ty = Math.toDegrees(Math.atan2(upCamera, forwardCamera)) + random.nextGaussian() * noiseDegrees;
                if (forwardCamera > 0 && Math.abs(tx) < 29 && Math.abs(ty) < 24) {
                    inFlight.add(new double[] { now, tx, ty });
                }
            }
            while (!inFlight.isEmpty() && inFlight.get(0)[0] + latencySeconds <= now) {
                double[] frame = inFlight.remove(0);
                double latencyMs = (now - frame[0]) * 1000;
                published++;
                table.getEntry("t2d").setDoubleArray(new double[] {
                    1, 1, latencyMs - 10, 10, frame[1], frame[2], 0, 0, 0.5, tagId, 0, 0, 0, 0, 0, 0, 0 });
                table.getEntry("tv").setDouble(1);
                table.getEntry("hb").setDouble(published);
            }
        }
    }

    /**
     * A Quest: tracks the truth from the pose it was last set to, 100 Hz frames stamped with
     * their publish time (capture + latency), like QuestNav.
     */
    final class FakeQuestNav extends QuestNav {
        final Transform3d offset;
        double latencySeconds = 0.03;
        double noiseMeters = 0.002;
        boolean connected = true;
        boolean tracking = true;
        /** Added to every frame from now on (a re-localization jump), field frame. */
        Translation2d jump = Translation2d.kZero;
        int setPoseCalls = 0;
        private Pose2d anchor;
        private Pose2d anchorTruthQuest;
        private double nextCapture;
        private final List<PoseFrame> inFlight = new ArrayList<>();
        private final List<PoseFrame> unread = new ArrayList<>();

        FakeQuestNav(Transform3d offset) {
            this.offset = offset;
            nextCapture = Timer.getFPGATimestamp();
            quests.add(this);
        }

        private Pose2d truthQuest() {
            return truth.transformBy(new Transform2d(offset.getTranslation().toTranslation2d(),
                    new Rotation2d(offset.getRotation().getZ())));
        }

        void update() {
            double now = Timer.getFPGATimestamp();
            if (now >= nextCapture) {
                nextCapture += 0.01;
                if (connected) {
                    Pose2d questTruth = truthQuest();
                    Pose2d pose = anchor == null ? questTruth : anchor.plus(questTruth.minus(anchorTruthQuest));
                    Translation2d position = pose.getTranslation().plus(jump)
                            .plus(new Translation2d(random.nextGaussian() * noiseMeters, random.nextGaussian() * noiseMeters));
                    inFlight.add(new PoseFrame(new Pose3d(new Translation3d(position.getX(), position.getY(), offset.getZ()),
                            new Rotation3d(0, 0, pose.getRotation().getRadians())), now + latencySeconds, now, 0, tracking));
                }
            }
            while (!inFlight.isEmpty() && inFlight.get(0).dataTimestamp() <= now) {
                PoseFrame frame = inFlight.remove(0);
                if (connected) {
                    unread.add(frame);
                }
            }
        }

        @Override
        public PoseFrame[] getAllUnreadPoseFrames() {
            PoseFrame[] frames = unread.toArray(new PoseFrame[0]);
            unread.clear();
            return frames;
        }

        @Override
        public boolean isConnected() {
            return connected;
        }

        @Override
        public OptionalInt getBatteryPercent() {
            return OptionalInt.of(90);
        }

        @Override
        public void setPose(Pose3d pose) {
            setPoseCalls++;
            anchor = pose.toPose2d();
            anchorTruthQuest = truthQuest();
            jump = Translation2d.kZero;
        }
    }

    /** A source that returns whatever is queued (for frames the real sources can't produce). */
    static final class FakeSource implements VisionSource {
        final List<TimestampedVisionMeasurement> queued = new ArrayList<>();
        private List<TimestampedVisionMeasurement> thisLoop = List.of();

        void add(Pose2d pose, double timestampSeconds, double std) {
            queued.add(new TimestampedVisionMeasurement(pose, timestampSeconds,
                    VecBuilder.fill(std, std, Double.POSITIVE_INFINITY)));
        }

        @Override
        public String getName() {
            return "fake";
        }

        @Override
        public boolean shouldUpdate() {
            return !thisLoop.isEmpty();
        }

        @Override
        public List<TimestampedVisionMeasurement> getPoseEstimates() {
            return thisLoop;
        }

        @Override
        public boolean isConnected() {
            return true;
        }

        @Override
        public void periodic() {
            thisLoop = new ArrayList<>(queued);
            queued.clear();
        }
    }
}
