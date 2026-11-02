package frc.robot.Path;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** The one {@link Sendable} published under {@code Path/<name>} (design doc sections 8.13, 9.11). */
final class PathTelemetry implements Sendable {

    private volatile String state = "RUNNING";
    private volatile int segment;
    private volatile double s;
    private volatile double eN;
    private volatile double vRef;
    private volatile double vMeas;
    private volatile double eThetaDeg;

    /** Creates an empty telemetry object. */
    PathTelemetry() {
    }

    /**
     * Stores the values the properties read; called at the end of each execute (8.14).
     *
     * @param state     state name: TRACK / CAPTURE_STOP / ROTATE / DWELL / CAPTURE_END / DONE / FAILED_*
     * @param segment   segment index at s* (5.9.3)
     * @param s         progress s* [m]
     * @param eN        cross-track error [m]
     * @param vRef      the speed law's v_cmd [m/s]
     * @param vMeas     abs(V_meas) [m/s]
     * @param eThetaDeg heading error [deg]
     */
    void update(String state, int segment, double s, double eN, double vRef, double vMeas, double eThetaDeg) {
        this.state = state;
        this.segment = segment;
        this.s = s;
        this.eN = eN;
        this.vRef = vRef;
        this.vMeas = vMeas;
        this.eThetaDeg = eThetaDeg;
    }

    /** Sets only the state name (a run that ended without a loop, e.g. FAILED_START_TOO_FAR). */
    void setState(String state) {
        this.state = state;
    }

    /** Adds the read-only keys state, segment, s, eN, vRef, vMeas, eThetaDeg (8.13). */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("state", () -> state, null);
        builder.addIntegerProperty("segment", () -> segment, null);
        builder.addDoubleProperty("s", () -> s, null);
        builder.addDoubleProperty("eN", () -> eN, null);
        builder.addDoubleProperty("vRef", () -> vRef, null);
        builder.addDoubleProperty("vMeas", () -> vMeas, null);
        builder.addDoubleProperty("eThetaDeg", () -> eThetaDeg, null);
    }
}
