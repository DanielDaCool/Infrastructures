package frc.robot.RobotPose.Estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;

/**
 * Reads odometry faster than the 50 Hz robot loop (250 Hz on a CAN FD bus such as a CANivore,
 * 100 Hz on the roboRIO's CAN 2.0 bus), so each odometry step covers 4 ms of motion instead
 * of 20 ms.
 *
 * <p>The thread only collects samples; the pose math still runs on the main thread. Each
 * cycle it:
 * <ol>
 * <li>Waits until every signal has a new value ({@code BaseStatusSignal.waitForAll}). On
 * CAN FD this returns as soon as the frames arrive, so all signals are sampled together.
 * CAN 2.0 can't wait on several signals, so there it sleeps one period and refreshes.</li>
 * <li>Calls the reader to build the sample from the values just received.</li>
 * <li>Stamps it with the FPGA time minus the signals' average CAN latency (close to when the
 * devices measured it) and queues it.</li>
 * </ol>
 * Each robot loop, {@link #getNewSamples()} (passed to {@code RobotPose.initialize}) hands
 * over everything queued since the last loop.
 *
 * <p>Example (the signals are clones, so the main thread refreshing the originals doesn't
 * interfere with this thread):
 * <pre>
 * StatusSignal&lt;Angle&gt; yaw = pigeon.getYaw().clone();
 * StatusSignal&lt;Angle&gt;[] drive = ...; // driveMotor.getPosition().clone() per module
 * StatusSignal&lt;Angle&gt;[] steer = ...; // steerMotor.getPosition().clone() per module
 *
 * PhoenixOdometryThread odometryThread = new PhoenixOdometryThread(canBus,
 *     () -&gt; new OdometryData(
 *         Rotation2d.fromDegrees(yaw.getValueAsDouble()),
 *         modulePositionsFrom(drive, steer)), // only reads getValueAsDouble()
 *     yaw, drive[0], steer[0], ..., drive[3], steer[3]);
 *
 * RobotPose.initialize(odometryThread::getNewSamples, ...);
 * </pre>
 */
public final class PhoenixOdometryThread extends Thread {
    /** Odometry frequency on a CAN FD bus (CANivore). */
    public static final double CAN_FD_FREQUENCY_HZ = 250.0;
    /** Odometry frequency on a CAN 2.0 bus (roboRIO). */
    public static final double CAN_2_FREQUENCY_HZ = 100.0;

    /**
     * Samples kept if nobody reads them (e.g. RobotPose isn't initialized). Dropping the
     * oldest loses no motion: samples hold total positions, so the next one still covers it.
     */
    private static final int MAX_QUEUED_SAMPLES = 100;

    private final BaseStatusSignal[] signals;
    private final Supplier<OdometryData> reader;
    private final boolean isCANFD;
    private final double frequencyHz;

    /** Samples waiting for the main thread. Guarded by {@code this}. */
    private List<OdometryData> queue = new ArrayList<>();
    /** Cycles where a signal failed to arrive in time (e.g. a device is unplugged). */
    private volatile int failedCycles;

    /**
     * Uses {@link #CAN_FD_FREQUENCY_HZ} on CAN FD, {@link #CAN_2_FREQUENCY_HZ} otherwise.
     *
     * @see #PhoenixOdometryThread(CANBus, double, Supplier, BaseStatusSignal...)
     */
    public PhoenixOdometryThread(CANBus canBus, Supplier<OdometryData> reader, BaseStatusSignal... signals) {
        this(canBus, canBus.isNetworkFD() ? CAN_FD_FREQUENCY_HZ : CAN_2_FREQUENCY_HZ, reader, signals);
    }

    /**
     * Sets the signals to {@code frequencyHz} and starts the thread.
     *
     * <p>If you call {@code optimizeBusUtilization} on these devices, call it after this, so
     * these signals keep their frequency.
     *
     * @param canBus      The bus the signals are on (all on the same bus).
     * @param frequencyHz How often to sample.
     * @param reader      Builds one sample from the signals' current values. Runs on this
     *                    thread, so it must only read the signals (e.g.
     *                    {@code getValueAsDouble()}), never refresh them or touch other
     *                    devices. The timestamp it sets is replaced.
     * @param signals     Every signal the reader uses (gyro yaw, drive and steer positions).
     *                    Pass clones ({@code getPosition().clone()}) so the main thread's
     *                    refreshes don't change them under the reader.
     */
    public PhoenixOdometryThread(CANBus canBus, double frequencyHz, Supplier<OdometryData> reader,
            BaseStatusSignal... signals) {
        super("PhoenixOdometryThread");
        this.signals = signals;
        this.reader = reader;
        this.isCANFD = canBus.isNetworkFD();
        this.frequencyHz = frequencyHz;

        BaseStatusSignal.setUpdateFrequencyForAll(frequencyHz, signals);
        setDaemon(true);
        start();
    }

    @Override
    public void run() {
        while (true) {
            StatusCode status;
            if (isCANFD) {
                // Twice the period, so one late frame doesn't count as a failure.
                status = BaseStatusSignal.waitForAll(2.0 / frequencyHz, signals);
            } else {
                try {
                    Thread.sleep((long) (1000.0 / frequencyHz));
                } catch (InterruptedException e) {
                    return;
                }
                status = BaseStatusSignal.refreshAll(signals);
            }

            if (!status.isOK()) {
                failedCycles++;
                continue;
            }

            double totalLatency = 0;
            for (BaseStatusSignal signal : signals) {
                totalLatency += signal.getTimestamp().getLatency();
            }
            double timestampSeconds = Timer.getFPGATimestamp() - totalLatency / signals.length;

            OdometryData sample = reader.get();
            OdometryData stamped = new OdometryData(timestampSeconds, sample.gyroAngle(), sample.swerveModules());
            synchronized (this) {
                if (queue.size() >= MAX_QUEUED_SAMPLES) {
                    queue.remove(0);
                }
                queue.add(stamped);
            }
        }
    }

    /**
     * @return Every sample since the last call, oldest first (about 5 per 20 ms loop at
     *         250 Hz). Pass {@code odometryThread::getNewSamples} to {@code RobotPose.initialize}.
     */
    public synchronized List<OdometryData> getNewSamples() {
        List<OdometryData> samples = queue;
        queue = new ArrayList<>();
        return samples;
    }

    /** @return The sampling frequency in Hz. */
    public double getFrequencyHz() {
        return frequencyHz;
    }

    /** @return How many cycles failed because a signal didn't arrive in time. */
    public int getFailedCycles() {
        return failedCycles;
    }
}
