package frc.robot.RobotPose.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotPose.Vision.visionConfigs.BaseVisionSourceConfig;

/**
 * Common base for every vision source: holds the config values and sets up the dashboard
 * entries ({@code vision/<name>} and {@code vision/<name>/field}).
 */
public abstract class BaseVisionSource implements VisionSource, Sendable {
    /**
     * Std dev growth with distance for tag cameras: std = this * distance^2 / sqrt(tags)
     * (meters, distance in meters). Measure: log the camera pose standing still at 1-5 m with 1
     * and 2+ tags and fit the spread.
     */
    public static final double STD_PER_METER_SQUARED = 0.008;
    /**
     * Heading error (radians) behind a tag camera's position: the pose is solved with our gyro
     * heading, so a heading error moves it by distance * this.
     */
    public static final double HEADING_STD_RADIANS = 0.01;

    /** Every source created so far, for {@link #setOnCreate}. */
    private static final List<VisionSource> createdSources = new ArrayList<>();
    /** Called with every source when it is created; see {@link #setOnCreate}. */
    private static Consumer<VisionSource> onCreate = source -> { };

    /** Name from the config. */
    protected final String name;
    /** Where the device is mounted relative to the robot center (robot to device). */
    protected Transform3d offset;
    /** Base measurement std devs from the config (x meters, y meters, theta radians). */
    protected Matrix<N3, N1> std;

    /** Robot pose from the newest frame (heading copied from the estimate). */
    protected Pose2d pose;

    private Field2d field;

    /** FPGA time the last used measurement was captured; NaN until one is used. */
    private double lastUsedTimestampSeconds = Double.NaN;
    /** Distance between the last used measurement and the estimate at its capture time. */
    private double lastErrorMeters;

    /** Copies the config and registers this source on SmartDashboard. */
    public BaseVisionSource(BaseVisionSourceConfig config) {
        name = config.name;
        offset = config.offset;
        std = config.std;
        field = new Field2d();
        pose = Pose2d.kZero;

        addLog();
        SmartDashboard.putData("vision/" + name, this);
        createdSources.add(this);
        onCreate.accept(this);
    }

    /**
     * Registers a callback that gets every vision source: the ones already created right away,
     * and every later one when it is created (from the base constructor, before the subclass
     * constructor has run, so only store the reference). For example
     * {@code BaseVisionSource.setOnCreate(ElasticGenerator.getInstance()::registerVisionSource)}.
     */
    public static synchronized void setOnCreate(Consumer<VisionSource> callback) {
        onCreate = callback;
        for (VisionSource source : createdSources) {
            callback.accept(source);
        }
    }

    /**
     * Std dev (meters) of a tag camera's x/y: {@code max(minStd, hypot(STD_PER_METER_SQUARED *
     * d^2 / sqrt(tags), d * HEADING_STD_RADIANS))}. Far tags and single tags are trusted less.
     *
     * @param minStd         The smallest std dev (the config x std dev).
     * @param distanceMeters Average distance to the tags.
     * @param tagCount       Tags in the frame (at least 1 is assumed).
     */
    public static double distanceStdDev(double minStd, double distanceMeters, int tagCount) {
        double std = STD_PER_METER_SQUARED * distanceMeters * distanceMeters / Math.sqrt(Math.max(tagCount, 1));
        return Math.max(minStd, Math.hypot(std, distanceMeters * HEADING_STD_RADIANS));
    }

    /** Puts dashboard entries. Subclasses can override to add their own (call super). */
    protected void addLog() {
        SmartDashboard.putData("vision/" + name + "/field", field);
    }

    public String getName() {
        return name;
    }

    @Override
    public void periodic() {
        field.setRobotPose(pose);
    }

    @Override
    public void onMeasurementUsed(TimestampedVisionMeasurement measurement, double errorMeters) {
        lastUsedTimestampSeconds = measurement.timestampSeconds();
        lastErrorMeters = errorMeters;
    }

    /**
     * Dashboard entries shared by every source:
     * <ul>
     * <li>{@code is Connected}: the device is talking to the robot.</li>
     * <li>{@code seconds since used}: age of the last used measurement (-1 if none yet).</li>
     * <li>{@code last error m}: how far the last used measurement was from the estimate. If
     * it stays large, the offset is wrong or the std devs are too small.</li>
     * </ul>
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("is Connected", () -> isConnected(), null);
        builder.addDoubleProperty("seconds since used", () -> Double.isNaN(lastUsedTimestampSeconds)
                ? -1 : Timer.getFPGATimestamp() - lastUsedTimestampSeconds, null);
        builder.addDoubleProperty("last error m", () -> lastErrorMeters, null);
    }
}
