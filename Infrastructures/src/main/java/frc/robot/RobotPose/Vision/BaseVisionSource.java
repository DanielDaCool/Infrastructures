package frc.robot.RobotPose.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotPose.Vision.visionConfigs.BaseVisionSourceConfig;

/**
 * Common base for every vision source: holds the config values and sets up the dashboard
 * entries ({@code vision/<name>} and {@code vision/<name>/field}).
 */
public abstract class BaseVisionSource implements VisionSource, Sendable {
    /** Name from the config. */
    protected final String name;
    /** Where the device is mounted relative to the robot center (robot to device). */
    protected Transform3d offset;
    /** Base measurement std devs from the config (x meters, y meters, theta radians). */
    protected Matrix<N3, N1> std;

    /** Robot pose from the newest frame (heading copied from the estimate). */
    protected Pose2d pose;

    private Field2d field;

    /** Copies the config and registers this source on SmartDashboard. */
    public BaseVisionSource(BaseVisionSourceConfig config) {
        name = config.name;
        offset = config.offset;
        std = config.std;
        field = new Field2d();
        pose = Pose2d.kZero;

        addLog();
        SmartDashboard.putData("vision/" + name, this);
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
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("is Connected", () -> isConnected(), null);
    }
}
