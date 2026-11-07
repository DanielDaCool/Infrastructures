package frc.robot.sim;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * Controls WPILib time in tests. While open, HAL timing is paused and only moves when
 * {@link #advance} is called, so {@link Timer#getFPGATimestamp()} and
 * {@link MathSharedStore#getTimestamp()} both return sim time [s].
 *
 * <p>Time is kept in whole microseconds (the HAL resolution) so the sim and WPILib never drift
 * apart. Use one clock at a time (HAL time is global); close it in {@code @AfterEach}.
 */
public final class SimClock implements AutoCloseable {
    private final MathShared previousMathShared;
    private long micros;
    private boolean closed;

    private SimClock() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        SimHooks.restartTiming(); // HAL time = 0
        micros = 0;
        previousMathShared = MathSharedStore.getMathShared();
        MathSharedStore.setMathShared(new MathShared() {
            @Override
            public void reportError(String error, StackTraceElement[] stackTrace) {
                System.err.println("[MathShared] " + error);
            }

            @Override
            public void reportUsage(MathUsageId id, int count) {}

            @Override
            public double getTimestamp() {
                return Timer.getFPGATimestamp();
            }
        });
    }

    /** Initialises the HAL, pauses and resets HAL time to 0 s, and routes MathSharedStore time to it. */
    public static SimClock start() {
        return new SimClock();
    }

    /** Advances WPILib time by {@code seconds} [s], rounded to whole microseconds. */
    public void advance(double seconds) {
        advanceMicros(Math.round(seconds * 1e6));
    }

    /** Advances WPILib time by {@code dMicros} [us] (must be >= 0). */
    public void advanceMicros(long dMicros) {
        if (dMicros < 0) throw new IllegalArgumentException("time cannot go back: " + dMicros);
        if (closed) throw new IllegalStateException("clock closed");
        micros += dMicros;
        SimulatorJNI.stepTiming(dMicros); // SimHooks.stepTiming((long)(s*1e6)) would truncate
    }

    /** Current sim time [s] as tracked by this clock. */
    public double now() {
        return micros * 1e-6;
    }

    /** Current sim time [us]. */
    public long nowMicros() {
        return micros;
    }

    /** Restores the previous MathShared and resumes real-time HAL timing. */
    @Override
    public void close() {
        if (closed) return;
        closed = true;
        MathSharedStore.setMathShared(previousMathShared);
        SimHooks.resumeTiming();
    }
}
