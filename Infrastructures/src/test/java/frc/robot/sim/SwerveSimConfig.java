package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * All parameters of {@link SwerveSim}. Plain mutable fields: start from {@link #defaults()} or
 * {@link #ideal()} and change what a test needs. Units are SI (m, s, kg, N, V, A), angles in rad,
 * CCW positive, robot frame x forward / y left (WPILib).
 *
 * <p>Every noise source has its own seed derived from {@link #seed}, and every non-ideal effect can
 * be switched off by setting its magnitude to 0 (or the boolean flag), see {@link #ideal()}.
 */
public final class SwerveSimConfig {

    // ---------------- geometry / mass ----------------
    /** Module positions on the robot [m], robot frame. Order = order of every per-module array. Default FL, FR, BL, BR. */
    public Translation2d[] modulePositions;
    /** Wheel radius [m]. */
    public double wheelRadius = 0.0508;
    /** Robot mass incl. bumpers and battery [kg]. */
    public double mass = 55.0;
    /** Yaw moment of inertia about the robot centre [kg m^2]. */
    public double momentOfInertia = 6.0;

    // ---------------- drive ----------------
    /** Drive motor model (one motor per module). */
    public DCMotor driveMotor = DCMotor.getKrakenX60(1);
    /** Drive reduction motor:wheel [-]. 6.03 matches SwerveAccelerationLimiter.FREE_SPEED. */
    public double driveGearing = 6.03;
    /** Rotational inertia of wheel + drive train, reflected to the wheel axle [kg m^2]. */
    public double driveInertia = 0.004;
    /** Drive stator current limit [A]; {@code <= 0} disables. */
    public double driveStatorLimit = 80.0;
    /** Drive supply current limit [A]; {@code <= 0} disables. */
    public double driveSupplyLimit = 60.0;
    /** Drive velocity loop gains, in wheel units: kS [V], kV [V/(m/s)], kA [V/(m/s^2)], kP [V/(m/s)]. NaN = derive kV/kA from the motor. */
    public double driveKS = 0.0, driveKV = Double.NaN, driveKA = Double.NaN, driveKP = 2.0;
    /**
     * Motor-internal friction: if true, a Coulomb torque of {@code Kt * freeCurrent} acts on each
     * motor so the unloaded motor spins at exactly the DCMotor free speed at nominal voltage.
     */
    public boolean motorFreeCurrentFriction = true;
    /** Rolling resistance coefficient [-] (force = c * normal force, opposes wheel rolling). */
    public double rollingResistance = 0.01;

    // ---------------- steer ----------------
    /** Steer motor model. */
    public DCMotor steerMotor = DCMotor.getKrakenX60(1);
    /** Steer reduction motor:module [-] (MK4i 150/7). */
    public double steerGearing = 150.0 / 7.0;
    /** Steer inertia about the azimuth axis, reflected to the module [kg m^2]. */
    public double steerInertia = 0.004;
    /** Steer stator current limit [A]; {@code <= 0} disables. */
    public double steerStatorLimit = 40.0;
    /** Steer position loop gains: kP [V/rad], kD [V/(rad/s)], kV (velocity FF) [V/(rad/s)] (NaN = from motor), kS [V]. */
    public double steerKP = 50.0, steerKD = 0.5, steerKV = Double.NaN, steerKS = 0.0;

    // ---------------- ground ----------------
    /** Wheel-ground friction coefficient [-]: max ground force per wheel = mu * N (friction circle). */
    public double frictionCoefficient = 1.2;
    /** Slip speed [m/s] at which friction saturates (linear below: regularised Coulomb). */
    public double slipSaturationSpeed = 0.05;
    /** Gravity [m/s^2]. */
    public double gravity = 9.81;

    // ---------------- battery ----------------
    /** Battery open-circuit voltage [V]. */
    public double batteryVoltage = 12.5;
    /** Battery + wiring resistance [ohm]; 0 = no sag. */
    public double batteryResistance = 0.015;
    /** Constant current of everything else on the robot [A]. */
    public double otherLoadCurrent = 2.0;

    // ---------------- timing ----------------
    /** Nominal robot loop period [s]. */
    public double loopPeriod = 0.02;
    /** Std dev of loop period jitter [s]; 0 = off. Actual dt clamped to [0.5, 2] * loopPeriod. */
    public double loopJitterStdDev = 0.001;
    /** Max physics substep [s] (must be <= 0.001). */
    public double maxSubstep = 0.00025;
    /** Command (CAN) latency from step() to the motor controllers [s]. */
    public double commandLatency = 0.010;
    /** Sensor latency: measured values are this old [s]. */
    public double sensorLatency = 0.005;

    // ---------------- sensors ----------------
    /** Drive encoder counts per motor revolution (TalonFX 2048); {@code <= 0} = no quantisation. */
    public double driveEncoderCpr = 2048;
    /** Steer absolute encoder counts per module revolution (CANcoder 4096); {@code <= 0} = none. */
    public double steerEncoderCpr = 4096;
    /** Std dev of drive position noise [m], drive velocity noise [m/s], steer angle noise [rad]. */
    public double drivePositionNoise = 0.0002, driveVelocityNoise = 0.01, steerAngleNoise = 0.002;
    /** Gyro yaw white noise [rad], rate noise [rad/s], drift [rad/s], scale error [-] (measured = (1+e) * true). */
    public double gyroYawNoise = 0.0005, gyroRateNoise = 0.002, gyroDriftRate = 0.0003, gyroScaleError = 0.001;

    // ---------------- modes ----------------
    /**
     * Perfect actuators: each module instantly takes the commanded state and the chassis moves by
     * exact rigid-body kinematics (least squares, WPILib forward kinematics). For oracle checks.
     */
    public boolean perfectActuators = false;
    /** Master seed; each noise source uses its own stream derived from it. */
    public long seed = 1;

    /** A typical 2026 Kraken X60, ~L2 4-module robot, 0.55 m track, with realistic noise and latency. */
    public static SwerveSimConfig defaults() {
        SwerveSimConfig c = new SwerveSimConfig();
        double h = 0.275;
        c.modulePositions = new Translation2d[] {
            new Translation2d(h, h), new Translation2d(h, -h),
            new Translation2d(-h, h), new Translation2d(-h, -h)};
        return c;
    }

    /** {@link #defaults()} with every noise, latency, jitter, quantisation, sag and gyro error off and perfect actuators. */
    public static SwerveSimConfig ideal() {
        return defaults().withIdealSensors().withPerfectActuators(true);
    }

    /** Turns off every noise source, latency, jitter, quantisation, gyro error and battery sag (keeps the physics). */
    public SwerveSimConfig withIdealSensors() {
        loopJitterStdDev = 0;
        commandLatency = 0;
        sensorLatency = 0;
        driveEncoderCpr = 0;
        steerEncoderCpr = 0;
        drivePositionNoise = driveVelocityNoise = steerAngleNoise = 0;
        gyroYawNoise = gyroRateNoise = gyroDriftRate = gyroScaleError = 0;
        batteryResistance = 0;
        return this;
    }

    /** Sets {@link #perfectActuators}. */
    public SwerveSimConfig withPerfectActuators(boolean on) {
        perfectActuators = on;
        return this;
    }

    /** Sets {@link #seed}. */
    public SwerveSimConfig withSeed(long s) {
        seed = s;
        return this;
    }

    /** Number of modules. */
    public int moduleCount() {
        return modulePositions.length;
    }

    /** Theoretical wheel free speed [m/s] = motor free speed / gearing * wheel radius. */
    public double driveFreeSpeed() {
        return driveMotor.freeSpeedRadPerSec / driveGearing * wheelRadius;
    }

    /** Drive kV used by the loop [V/(m/s) at the wheel]. */
    public double effectiveDriveKV() {
        return Double.isNaN(driveKV) ? driveGearing / (driveMotor.KvRadPerSecPerVolt * wheelRadius) : driveKV;
    }

    /** Drive kA used by the loop [V/(m/s^2) at the wheel]: voltage for 1 m/s^2 of a mass/n share. */
    public double effectiveDriveKA() {
        if (!Double.isNaN(driveKA)) return driveKA;
        double forcePerVolt = driveGearing * driveMotor.KtNMPerAmp / (driveMotor.rOhms * wheelRadius); // [N/V] at stall
        double massShare = mass / moduleCount() + driveInertia / (wheelRadius * wheelRadius);
        return massShare / forcePerVolt;
    }

    /** Steer kV used by the loop [V/(rad/s) at the module]. */
    public double effectiveSteerKV() {
        return Double.isNaN(steerKV) ? steerGearing / steerMotor.KvRadPerSecPerVolt : steerKV;
    }

    /** Deep copy. */
    public SwerveSimConfig copy() {
        SwerveSimConfig c = new SwerveSimConfig();
        for (java.lang.reflect.Field f : SwerveSimConfig.class.getFields()) {
            if (java.lang.reflect.Modifier.isStatic(f.getModifiers())) continue;
            try {
                f.set(c, f.get(this));
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(e);
            }
        }
        c.modulePositions = modulePositions.clone();
        return c;
    }
}
