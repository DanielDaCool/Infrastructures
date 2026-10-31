package frc.robot.Kinematics;

/**
 * Settings for {@link DemaciaKinematics} and {@link SwerveAccelerationLimiter}, one per drivetrain.
 *
 * <p>Every setting has a default (placeholders: tune them on the robot). Change only what differs:
 *
 * <pre>{@code
 * KinematicsConfig config = new KinematicsConfig()
 *     .withMaxAllowedModuleVelocity(4.2) // [m/s]
 *     .withLatency(0.03)                 // [s]
 *     .withFreeSpeed(5.0);               // [m/s]
 * DemaciaKinematics kinematics = new DemaciaKinematics(config, modulePositions);
 * SwerveAccelerationLimiter limiter = new SwerveAccelerationLimiter(config);
 * }</pre>
 *
 * <p>The kinematics and the limiter copy the values when they are created, so change the config
 * before creating them.
 */
public class KinematicsConfig {

    // Kinematics

    /** Robot loop period, the discretize time and one feed-forward step [s]. */
    public double cycleDt = 0.02;
    /**
     * CAN/motor command latency + gyro latency, no half loop (the discretize over
     * {@code cycleDt + 2 * latency} already adds it). Placeholder: tune on the robot [s].
     */
    public double latency = 0.035;
    /** Top speed of a module, the desaturation limit [m/s]. */
    public double maxAllowedModuleVelocity = 4.5;
    /** Below this the robot stands and keeps its module angles [m/s and rad/s]. */
    public double minVelocity = 0.01;
    /**
     * Safety clamp of the drive acceleration feed-forward, above traction (the limiter sets the
     * real limit) [m/s^2].
     */
    public double maxDriveAccel = 20.0;
    /** Clamp of the steer velocity feed-forward: free steer speed [rad/s]. */
    public double maxSteerRate = 12.0;

    // Acceleration limiter (placeholders: tune on the robot)

    /** Forward acceleration limit at rest [m/s^2]. */
    public double maxForwardAccel = 10.0;
    /**
     * Where the drive motors run out of torque: motor free rpm / gear ratio * wheel circumference
     * (default: Kraken X60 6000 rpm, no FOC, / 6.03 * 4" wheel) [m/s].
     */
    public double freeSpeed = 5.3;
    /** Traction limit: total velocity change [m/s^2]. */
    public double maxSkidAccel = 8.0;
    /** Tipping limit along the robot's front/back [m/s^2]. */
    public double maxTiltAccelFront = 12.0;
    /** Tipping limit along the robot's left/right [m/s^2]. */
    public double maxTiltAccelSide = 12.0;
    /** Longest step of one limiter call: a slow loop still moves at most one step [s]. */
    public double maxDt = 0.02;

    public KinematicsConfig withCycleDt(double cycleDt) {
        this.cycleDt = cycleDt;
        return this;
    }

    public KinematicsConfig withLatency(double latency) {
        this.latency = latency;
        return this;
    }

    public KinematicsConfig withMaxAllowedModuleVelocity(double maxAllowedModuleVelocity) {
        this.maxAllowedModuleVelocity = maxAllowedModuleVelocity;
        return this;
    }

    public KinematicsConfig withMinVelocity(double minVelocity) {
        this.minVelocity = minVelocity;
        return this;
    }

    public KinematicsConfig withMaxDriveAccel(double maxDriveAccel) {
        this.maxDriveAccel = maxDriveAccel;
        return this;
    }

    public KinematicsConfig withMaxSteerRate(double maxSteerRate) {
        this.maxSteerRate = maxSteerRate;
        return this;
    }

    public KinematicsConfig withMaxForwardAccel(double maxForwardAccel) {
        this.maxForwardAccel = maxForwardAccel;
        return this;
    }

    public KinematicsConfig withFreeSpeed(double freeSpeed) {
        this.freeSpeed = freeSpeed;
        return this;
    }

    public KinematicsConfig withMaxSkidAccel(double maxSkidAccel) {
        this.maxSkidAccel = maxSkidAccel;
        return this;
    }

    public KinematicsConfig withMaxTiltAccel(double front, double side) {
        this.maxTiltAccelFront = front;
        this.maxTiltAccelSide = side;
        return this;
    }

    public KinematicsConfig withMaxDt(double maxDt) {
        this.maxDt = maxDt;
        return this;
    }
}
