package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.Random;

/**
 * Deterministic swerve-drive physics simulation for tests (ground truth + realistic sensors).
 *
 * <p>Model per module: steer position loop (PD + velocity FF) and drive velocity loop
 * (kS + kV + kA FF + kP) running every substep like an on-board motor controller, a DCMotor model
 * with gearing, stator/supply current limits and a shared battery with internal resistance;
 * wheel-ground contact with a friction circle (mu * N, regularised below
 * {@link SwerveSimConfig#slipSaturationSpeed}) so wheels slip when the motor torque exceeds
 * traction. The chassis is a 2D rigid body (mass, yaw inertia) driven by the four contact forces.
 * Commands are applied after {@link SwerveSimConfig#commandLatency}; sensors report values that are
 * {@link SwerveSimConfig#sensorLatency} old with quantisation and noise.
 *
 * <p>Units: SI, angles rad CCW positive, field frame x/y as WPILib (blue origin), robot frame
 * x forward / y left. Module order = {@link SwerveSimConfig#modulePositions}.
 *
 * <p>Usage: {@code sim.step(states, accelFF, steerVelFF)} once per robot loop, then read sensors.
 * If a {@link SimClock} is attached, each step advances WPILib time by the same (jittered) dt.
 */
public final class SwerveSim {
    private static final double HISTORY_SECONDS = 3.0;

    private final SwerveSimConfig cfg;
    private final int n;
    private final SimClock clock;
    private final SwerveDriveKinematics kinematics;
    private final Random rngJitter, rngGyro;
    private final Random[] rngEnc;

    // derived motor constants
    private final double driveKV, driveKA, driveKS, steerKV;
    private final double normalForce;

    // time
    private long timeUs;
    private long lastDtUs;

    // chassis truth (field frame)
    private double x, y, theta, vx, vy, omega;

    // module truth
    private final double[] steerAngle, steerRate, wheelOmega, wheelAngle, slip;
    private final double[] driveVolts, driveStator, driveSupply, steerVolts, steerStator;
    private double supplyTotal, busVoltage;

    // active command
    private final double[] cmdSpeed, cmdAngle, cmdAccel, cmdSteerVel;
    private boolean cmdValid;
    private int nanCommands;
    private final ArrayDeque<Command> pending = new ArrayDeque<>();

    // history of truth (per substep)
    private final ArrayDeque<Snapshot> history = new ArrayDeque<>();

    // latest measurements
    private SwerveModuleState[] measStates;
    private SwerveModulePosition[] measPositions;
    private double measYaw, measRate, measTime;
    private double gyroOffset;

    private record Command(double applyUs, double[] speed, double[] angle, double[] accel, double[] steerVel, boolean valid) {}

    private record Snapshot(double t, double x, double y, double theta, double omega, double[] dist, double[] vel, double[] steer) {}

    /** Sim with its own time starting at 0 s (WPILib time not controlled). */
    public SwerveSim(SwerveSimConfig config) {
        this(config, null);
    }

    /**
     * Sim that advances {@code clock} (may be null) every step. Sim time starts at the clock's time.
     * The config is copied; later changes to it have no effect.
     */
    public SwerveSim(SwerveSimConfig config, SimClock clock) {
        this.cfg = config.copy();
        if (cfg.maxSubstep <= 0 || cfg.maxSubstep > 0.001) throw new IllegalArgumentException("maxSubstep must be in (0, 1 ms]");
        this.n = cfg.moduleCount();
        this.clock = clock;
        this.kinematics = new SwerveDriveKinematics(cfg.modulePositions);
        long s = cfg.seed * 0x9E3779B97F4A7C15L;
        rngJitter = new Random(s + 1);
        rngGyro = new Random(s + 2);
        rngEnc = new Random[n];
        for (int i = 0; i < n; i++) rngEnc[i] = new Random(s + 100 + i);
        driveKV = cfg.effectiveDriveKV();
        driveKA = cfg.effectiveDriveKA();
        // kS default: voltage that overcomes the motor's own friction (free current), at the wheel
        driveKS = Double.isNaN(cfg.driveKS) ? cfg.driveMotor.rOhms * cfg.driveMotor.freeCurrentAmps : cfg.driveKS;
        steerKV = cfg.effectiveSteerKV();
        normalForce = cfg.mass * cfg.gravity / n;
        steerAngle = new double[n];
        steerRate = new double[n];
        wheelOmega = new double[n];
        wheelAngle = new double[n];
        slip = new double[n];
        driveVolts = new double[n];
        driveStator = new double[n];
        driveSupply = new double[n];
        steerVolts = new double[n];
        steerStator = new double[n];
        cmdSpeed = new double[n];
        cmdAngle = new double[n];
        cmdAccel = new double[n];
        cmdSteerVel = new double[n];
        cmdValid = false; // motors neutral until the first command arrives
        busVoltage = cfg.batteryVoltage;
        timeUs = clock == null ? 0 : clock.nowMicros();
        recordSnapshot();
        updateMeasurements();
    }

    // ------------------------------------------------------------------ stepping

    /** {@link #step(SwerveModuleState[], double[], double[])} without feed-forwards. */
    public void step(SwerveModuleState[] commanded) {
        step(commanded, null, null);
    }

    /**
     * Advances the world by one robot loop ({@link SwerveSimConfig#loopPeriod} plus jitter).
     *
     * @param commanded   module setpoints, speed [m/s] and angle (robot frame, CCW+), module order;
     *                    applied as given (no optimisation). A null/NaN entry sets that module neutral.
     * @param driveAccelFF drive acceleration feed-forward per module [m/s^2], or null
     * @param steerVelFF   steer velocity feed-forward per module [rad/s, CCW+], or null
     */
    public void step(SwerveModuleState[] commanded, double[] driveAccelFF, double[] steerVelFF) {
        if (commanded.length != n) throw new IllegalArgumentException("expected " + n + " module states");
        double[] sp = new double[n], an = new double[n], ac = new double[n], sv = new double[n];
        boolean valid = true;
        for (int i = 0; i < n; i++) {
            SwerveModuleState st = commanded[i];
            sp[i] = st == null ? Double.NaN : st.speedMetersPerSecond;
            an[i] = st == null || st.angle == null ? Double.NaN : st.angle.getRadians();
            ac[i] = driveAccelFF == null ? 0 : driveAccelFF[i];
            sv[i] = steerVelFF == null ? 0 : steerVelFF[i];
            if (!Double.isFinite(sp[i]) || !Double.isFinite(an[i]) || !Double.isFinite(ac[i]) || !Double.isFinite(sv[i])) valid = false;
        }
        if (!valid) nanCommands++;
        pending.addLast(new Command(timeUs + cfg.commandLatency * 1e6, sp, an, ac, sv, valid));

        double dt = cfg.loopPeriod;
        if (cfg.loopJitterStdDev > 0) {
            dt = MathUtil.clamp(dt + cfg.loopJitterStdDev * rngJitter.nextGaussian(), 0.5 * cfg.loopPeriod, 2 * cfg.loopPeriod);
        }
        long dtUs = Math.max(1, Math.round(dt * 1e6));
        int subs = (int) Math.ceil(dtUs * 1e-6 / cfg.maxSubstep - 1e-9);
        double hUs = (double) dtUs / subs;
        double h = hUs * 1e-6;
        double startUs = timeUs;
        for (int k = 0; k < subs; k++) {
            double subStartUs = startUs + k * hUs;
            while (!pending.isEmpty() && pending.peekFirst().applyUs() <= subStartUs + 1e-6) applyCommand(pending.pollFirst());
            if (cfg.perfectActuators) perfectSubstep(h);
            else physicsSubstep(h);
            subTimeForSnapshot = (startUs + (k + 1) * hUs) * 1e-6;
            recordSnapshot();
        }
        timeUs += dtUs;
        lastDtUs = dtUs;
        if (clock != null) clock.advanceMicros(dtUs);
        updateMeasurements();
    }

    private double subTimeForSnapshot = 0;

    private void applyCommand(Command c) {
        System.arraycopy(c.speed(), 0, cmdSpeed, 0, n);
        System.arraycopy(c.angle(), 0, cmdAngle, 0, n);
        System.arraycopy(c.accel(), 0, cmdAccel, 0, n);
        System.arraycopy(c.steerVel(), 0, cmdSteerVel, 0, n);
        cmdValid = c.valid();
    }

    private boolean moduleCommanded(int i) {
        return cmdValid && Double.isFinite(cmdSpeed[i]) && Double.isFinite(cmdAngle[i]);
    }

    /** Perfect actuators: modules take the command instantly, chassis moves by exact kinematics. */
    private void perfectSubstep(double h) {
        SwerveModuleState[] st = new SwerveModuleState[n];
        for (int i = 0; i < n; i++) {
            if (moduleCommanded(i)) {
                steerRate[i] = 0;
                steerAngle[i] = cmdAngle[i];
                wheelOmega[i] = cmdSpeed[i] / cfg.wheelRadius;
            } else {
                wheelOmega[i] = 0;
            }
            wheelAngle[i] += wheelOmega[i] * h;
            slip[i] = 0;
            st[i] = new SwerveModuleState(wheelOmega[i] * cfg.wheelRadius, new Rotation2d(steerAngle[i]));
        }
        ChassisSpeeds rs = kinematics.toChassisSpeeds(st);
        Pose2d p = new Pose2d(x, y, new Rotation2d(theta)).exp(
                new Twist2d(rs.vxMetersPerSecond * h, rs.vyMetersPerSecond * h, rs.omegaRadiansPerSecond * h));
        x = p.getX();
        y = p.getY();
        theta += rs.omegaRadiansPerSecond * h;
        double c = Math.cos(theta), s = Math.sin(theta);
        vx = c * rs.vxMetersPerSecond - s * rs.vyMetersPerSecond;
        vy = s * rs.vxMetersPerSecond + c * rs.vyMetersPerSecond;
        omega = rs.omegaRadiansPerSecond;
        busVoltage = cfg.batteryVoltage;
    }

    private void physicsSubstep(double h) {
        busVoltage = Math.max(0, cfg.batteryVoltage - cfg.batteryResistance * (supplyTotal + cfg.otherLoadCurrent));
        double vbus = busVoltage;
        double c = Math.cos(theta), s = Math.sin(theta);
        double vrx = c * vx + s * vy, vry = -s * vx + c * vy; // robot-frame chassis velocity
        double fx = 0, fy = 0, tau = 0;
        double supply = 0;
        DCMotor dm = cfg.driveMotor, sm = cfg.steerMotor;
        double r = cfg.wheelRadius, gd = cfg.driveGearing, gs = cfg.steerGearing;
        for (int i = 0; i < n; i++) {
            boolean on = moduleCommanded(i);
            // ---- steer: position loop on the module angle
            double vs = 0;
            if (on) {
                double err = MathUtil.angleModulus(cmdAngle[i] - steerAngle[i]);
                double ff = cmdSteerVel[i];
                vs = cfg.steerKP * err + cfg.steerKD * (ff - steerRate[i]) + steerKV * ff
                        + (Math.abs(err) > 1e-3 ? Math.copySign(cfg.steerKS, err) : 0);
            }
            double steerMotorSpeed = gs * steerRate[i];
            vs = limitVoltage(vs, vbus, steerMotorSpeed, sm, cfg.steerStatorLimit, 0);
            steerVolts[i] = vs;
            steerStator[i] = (vs - steerMotorSpeed / sm.KvRadPerSecPerVolt) / sm.rOhms;
            supply += Math.abs(vbus > 0 ? steerStator[i] * vs / vbus : 0);
            double sTorquePerVolt = gs * sm.KtNMPerAmp / sm.rOhms;
            double sDamping = gs * gs * sm.KtNMPerAmp / (sm.rOhms * sm.KvRadPerSecPerVolt);
            double sFric = cfg.motorFreeCurrentFriction ? gs * sm.KtNMPerAmp * sm.freeCurrentAmps * Math.tanh(steerMotorSpeed) : 0;
            steerRate[i] = (cfg.steerInertia * steerRate[i] + h * (sTorquePerVolt * vs - sFric))
                    / (cfg.steerInertia + h * sDamping);
            steerAngle[i] += h * steerRate[i];

            // ---- drive: velocity loop on the wheel surface speed
            double vWheel = wheelOmega[i] * r;
            double vd = 0;
            if (on) {
                double target = cmdSpeed[i];
                vd = (Math.abs(target) > 1e-9 ? Math.copySign(driveKS, target) : 0) + driveKV * target
                        + driveKA * cmdAccel[i] + cfg.driveKP * (target - vWheel);
            }
            double driveMotorSpeed = gd * wheelOmega[i];
            vd = limitVoltage(vd, vbus, driveMotorSpeed, dm, cfg.driveStatorLimit, cfg.driveSupplyLimit);
            driveVolts[i] = vd;
            driveStator[i] = (vd - driveMotorSpeed / dm.KvRadPerSecPerVolt) / dm.rOhms;
            driveSupply[i] = vbus > 0 ? driveStator[i] * vd / vbus : 0;
            supply += Math.abs(driveSupply[i]);

            // ---- ground contact (robot frame)
            double px = cfg.modulePositions[i].getX(), py = cfg.modulePositions[i].getY();
            double vpx = vrx - omega * py, vpy = vry + omega * px;
            double ex = Math.cos(steerAngle[i]), ey = Math.sin(steerAngle[i]);
            double ux = vpx - wheelOmega[i] * r * ex, uy = vpy - wheelOmega[i] * r * ey;
            double fmax = cfg.frictionCoefficient * normalForce;
            double g = fmax / Math.max(Math.hypot(ux, uy), cfg.slipSaturationSpeed);
            double vLong = vpx * ex + vpy * ey;
            double dTorquePerVolt = gd * dm.KtNMPerAmp / dm.rOhms;
            double dDamping = gd * gd * dm.KtNMPerAmp / (dm.rOhms * dm.KvRadPerSecPerVolt);
            double dFric = cfg.motorFreeCurrentFriction ? gd * dm.KtNMPerAmp * dm.freeCurrentAmps * Math.tanh(driveMotorSpeed) : 0;
            double rolling = cfg.rollingResistance * normalForce * r * Math.tanh(wheelOmega[i] / 0.5);
            // implicit in the wheel speed for motor back-EMF damping and slip stiffness (stable at any substep)
            wheelOmega[i] = (cfg.driveInertia * wheelOmega[i] + h * (dTorquePerVolt * vd - dFric - rolling + r * g * vLong))
                    / (cfg.driveInertia + h * (dDamping + r * r * g));
            wheelAngle[i] += h * wheelOmega[i];
            ux = vpx - wheelOmega[i] * r * ex;
            uy = vpy - wheelOmega[i] * r * ey;
            double fix = -g * ux, fiy = -g * uy;
            double fm = Math.hypot(fix, fiy);
            if (fm > fmax) {
                fix *= fmax / fm;
                fiy *= fmax / fm;
            }
            slip[i] = Math.hypot(ux, uy);
            fx += fix;
            fy += fiy;
            tau += px * fiy - py * fix;
        }
        supplyTotal = supply;
        // ---- rigid body (semi-implicit Euler)
        double ax = (c * fx - s * fy) / cfg.mass, ay = (s * fx + c * fy) / cfg.mass;
        vx += h * ax;
        vy += h * ay;
        omega += h * tau / cfg.momentOfInertia;
        x += h * vx;
        y += h * vy;
        theta += h * omega;
    }

    /** Clamps a motor voltage to the bus, then to the stator and supply current limits [V]. */
    private static double limitVoltage(double v, double vbus, double motorSpeed, DCMotor m, double statorLimit, double supplyLimit) {
        v = MathUtil.clamp(v, -vbus, vbus);
        double emf = motorSpeed / m.KvRadPerSecPerVolt;
        if (statorLimit > 0) {
            double i = (v - emf) / m.rOhms;
            if (Math.abs(i) > statorLimit) v = emf + Math.copySign(statorLimit, i) * m.rOhms;
            v = MathUtil.clamp(v, -vbus, vbus);
        }
        if (supplyLimit > 0 && vbus > 0) {
            double i = (v - emf) / m.rOhms;
            if (i * v / vbus > supplyLimit) { // motoring beyond the limit: solve v (v - emf) = L vbus R
                double disc = emf * emf + 4 * supplyLimit * vbus * m.rOhms;
                double r1 = (emf + Math.sqrt(disc)) / 2, r2 = (emf - Math.sqrt(disc)) / 2;
                v = v > 0 ? r1 : r2;
            }
        }
        return v;
    }

    // ------------------------------------------------------------------ history / sensors

    private void recordSnapshot() {
        double[] dist = new double[n], vel = new double[n], st = new double[n];
        for (int i = 0; i < n; i++) {
            dist[i] = wheelAngle[i] * cfg.wheelRadius;
            vel[i] = wheelOmega[i] * cfg.wheelRadius;
            st[i] = steerAngle[i];
        }
        double t = history.isEmpty() ? timeUs * 1e-6 : subTimeForSnapshot;
        history.addLast(new Snapshot(t, x, y, theta, omega, dist, vel, st));
        double keep = HISTORY_SECONDS + cfg.sensorLatency;
        while (history.size() > 2 && history.peekFirst().t() < t - keep) history.pollFirst();
    }

    /** Latest snapshot with time <= t (oldest one if none). */
    private Snapshot snapshotAt(double t) {
        Iterator<Snapshot> it = history.descendingIterator();
        Snapshot s = null;
        while (it.hasNext()) {
            s = it.next();
            if (s.t() <= t + 1e-9) return s;
        }
        return s;
    }

    private void updateMeasurements() {
        Snapshot s = snapshotAt(getTime() - cfg.sensorLatency);
        measTime = s.t();
        measStates = new SwerveModuleState[n];
        measPositions = new SwerveModulePosition[n];
        double qd = cfg.driveEncoderCpr > 0 ? 2 * Math.PI * cfg.wheelRadius / (cfg.driveGearing * cfg.driveEncoderCpr) : 0;
        double qs = cfg.steerEncoderCpr > 0 ? 2 * Math.PI / cfg.steerEncoderCpr : 0;
        for (int i = 0; i < n; i++) {
            Random rng = rngEnc[i];
            double pos = quantize(s.dist()[i], qd) + gauss(rng, cfg.drivePositionNoise);
            double vel = s.vel()[i] + gauss(rng, cfg.driveVelocityNoise);
            double ang = MathUtil.angleModulus(quantize(s.steer()[i], qs) + gauss(rng, cfg.steerAngleNoise));
            Rotation2d rot = new Rotation2d(ang);
            measStates[i] = new SwerveModuleState(vel, rot);
            measPositions[i] = new SwerveModulePosition(pos, rot);
        }
        measYaw = gyroNoNoise(s) + gauss(rngGyro, cfg.gyroYawNoise);
        measRate = (1 + cfg.gyroScaleError) * s.omega() + cfg.gyroDriftRate + gauss(rngGyro, cfg.gyroRateNoise);
    }

    private double gyroNoNoise(Snapshot s) {
        return gyroOffset + (1 + cfg.gyroScaleError) * s.theta() + cfg.gyroDriftRate * s.t();
    }

    private static double quantize(double v, double q) {
        return q > 0 ? Math.floor(v / q) * q : v;
    }

    private static double gauss(Random r, double sigma) {
        return sigma > 0 ? sigma * r.nextGaussian() : 0;
    }

    // ------------------------------------------------------------------ resets

    /**
     * Teleports the robot to {@code pose} [m, rad] at rest (chassis and wheels stopped, history
     * cleared). The gyro is a physical sensor: its reading stays continuous; use {@link #setGyroYaw}.
     * Drive encoder positions keep counting.
     */
    public void resetPose(Pose2d pose) {
        double before = gyroNoNoise(snapshotAt(getTime() - cfg.sensorLatency));
        x = pose.getX();
        y = pose.getY();
        theta = pose.getRotation().getRadians();
        vx = vy = omega = 0;
        for (int i = 0; i < n; i++) {
            wheelOmega[i] = 0;
            steerRate[i] = 0;
            slip[i] = 0;
        }
        history.clear();
        recordSnapshot();
        gyroOffset += before - gyroNoNoise(history.peekLast());
        updateMeasurements();
    }

    /** Re-zeroes the gyro so that it reads {@code yaw} now (noise-free part) [rad, CCW+]. */
    public void setGyroYaw(Rotation2d yaw) {
        gyroOffset += yaw.getRadians() - gyroNoNoise(snapshotAt(getTime() - cfg.sensorLatency));
        updateMeasurements();
    }

    // ------------------------------------------------------------------ truth

    /** Sim time [s]. Equals WPILib time when a {@link SimClock} is attached. */
    public double getTime() {
        return timeUs * 1e-6;
    }

    /** Duration of the last step, incl. jitter [s]. */
    public double getLastDt() {
        return lastDtUs * 1e-6;
    }

    /** The config in use (a copy; do not modify). */
    public SwerveSimConfig getConfig() {
        return cfg;
    }

    /** True field pose [m, rad]; the rotation is continuous (not wrapped). */
    public Pose2d getTruePose() {
        return new Pose2d(x, y, new Rotation2d(theta));
    }

    /** True yaw, continuous [rad, CCW+]. */
    public Rotation2d getTrueYaw() {
        return new Rotation2d(theta);
    }

    /** True field-relative chassis speeds [m/s, rad/s]. */
    public ChassisSpeeds getTrueFieldSpeeds() {
        return new ChassisSpeeds(vx, vy, omega);
    }

    /** True robot-relative chassis speeds [m/s, rad/s]. */
    public ChassisSpeeds getTrueRobotSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getTrueFieldSpeeds(), new Rotation2d(theta));
    }

    /** True module states: wheel surface speed [m/s] and module angle wrapped to (-pi, pi] [rad], module order. */
    public SwerveModuleState[] getTrueModuleStates() {
        SwerveModuleState[] r = new SwerveModuleState[n];
        for (int i = 0; i < n; i++) r[i] = new SwerveModuleState(wheelOmega[i] * cfg.wheelRadius, new Rotation2d(MathUtil.angleModulus(steerAngle[i])));
        return r;
    }

    /** True module positions: wheel distance rolled [m] and module angle [rad]. */
    public SwerveModulePosition[] getTrueModulePositions() {
        SwerveModulePosition[] r = new SwerveModulePosition[n];
        for (int i = 0; i < n; i++) r[i] = new SwerveModulePosition(wheelAngle[i] * cfg.wheelRadius, new Rotation2d(MathUtil.angleModulus(steerAngle[i])));
        return r;
    }

    /** Contact-patch slip speed per module [m/s] (0 = pure rolling). */
    public double[] getWheelSlip() {
        return slip.clone();
    }

    /** True pose at a past time [s] (linear interpolation of the substep history, last ~3 s; clamped to it). */
    public Pose2d getTruePoseAt(double t) {
        Iterator<Snapshot> it = history.descendingIterator();
        Snapshot later = null;
        while (it.hasNext()) {
            Snapshot s = it.next();
            if (s.t() <= t) {
                if (later == null) return new Pose2d(s.x(), s.y(), new Rotation2d(s.theta()));
                double a = (t - s.t()) / (later.t() - s.t());
                return new Pose2d(s.x() + a * (later.x() - s.x()), s.y() + a * (later.y() - s.y()),
                        new Rotation2d(s.theta() + a * (later.theta() - s.theta())));
            }
            later = s;
        }
        return new Pose2d(later.x(), later.y(), new Rotation2d(later.theta()));
    }

    /** Battery bus voltage during the last substep [V]. */
    public double getBusVoltage() {
        return busVoltage;
    }

    /** Drive motor voltages [V], stator currents [A], supply currents [A] (last substep), module order. */
    public double[] getDriveVoltages() {
        return driveVolts.clone();
    }

    /** See {@link #getDriveVoltages()}. */
    public double[] getDriveStatorCurrents() {
        return driveStator.clone();
    }

    /** See {@link #getDriveVoltages()}. */
    public double[] getDriveSupplyCurrents() {
        return driveSupply.clone();
    }

    /** Steer motor voltages [V] (last substep), module order. */
    public double[] getSteerVoltages() {
        return steerVolts.clone();
    }

    /** Number of step() calls that contained a null/NaN/infinite command (those modules went neutral). */
    public int getInvalidCommandCount() {
        return nanCommands;
    }

    // ------------------------------------------------------------------ sensors

    /** Measured module states (latency, velocity noise, angle quantisation + noise) [m/s, rad]. */
    public SwerveModuleState[] getMeasuredModuleStates() {
        return measStates.clone();
    }

    /** Measured module positions (latency, encoder quantisation + noise) [m, rad]. */
    public SwerveModulePosition[] getMeasuredModulePositions() {
        return measPositions.clone();
    }

    /** Measured gyro yaw, continuous (latency, offset, scale error, drift, noise) [rad, CCW+]. */
    public Rotation2d getGyroYaw() {
        return new Rotation2d(measYaw);
    }

    /** Measured gyro yaw rate [rad/s, CCW+]. */
    public double getGyroRate() {
        return measRate;
    }

    /** Time at which the current measurements were true (= getTime() - sensorLatency, substep resolution) [s]. */
    public double getMeasurementTimestamp() {
        return measTime;
    }
}
