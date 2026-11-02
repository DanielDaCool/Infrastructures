package frc.robot.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The follower's state machine and the translation and heading laws; the finish test; the
 * follower constants (design doc sections 8.2 to 8.7). Owns the projector, speed law and event
 * schedule of one run. {@link FollowPathCommand} delegates to it.
 */
final class Tracker {

    /** Follower states (8.14). */
    enum State { TRACK, CAPTURE_STOP, ROTATE, DWELL, CAPTURE_END, DONE }

    /** Nominal loop period [s] (8.2.2). */
    static final double DT_NOM = 0.02;
    /** Loop period clamp [s] (8.2.2). */
    static final double DT_MIN = 0.005;
    /** Loop period clamp [s] (8.2.2). */
    static final double DT_MAX = 0.10;
    /** Largest hand-over distance d_h [m] (8.3.7); d_h = min(this, jerkMax*DT_NOM/kP^2). */
    static final double D_H_MAX = 0.05;
    /** Correction clamp c_max [m/s] (8.3.8). */
    static final double C_MAX = 1.0;
    /** Minimum lookahead of the projection window [m] (8.2.3). */
    static final double AHEAD_MIN = 0.15;
    /** Window lookahead in loops (8.2.3). */
    static final int K_AHEAD = 3;
    /** Maximum lookahead of the projection window [m] (8.2.3). */
    static final double AHEAD_MAX = 0.50;
    /** Window look-back eps_back [m] (8.2.4). */
    static final double EPS_BACK = 0.03;
    /** Minimum arc window beyond the arc start, {@code max(this, (pi/2) r_j)} [m] (8.2.3). */
    static final double ARC_WINDOW_MIN = 0.05;
    /** Face-target freeze radius r_face_min [m] (6.9, 8.5.3). */
    static final double R_FACE_MIN = 0.50;
    /** Stop speed [m/s] (7.10.3, 8.6). */
    static final double STOP_SPEED = 0.1;
    /** Stop ROTATE entry and exit angle, 3 deg [rad] (6.7, 8.6). */
    static final double STOP_ANG_TOL = Math.toRadians(3.0);
    /** Path shorter than this starts in CAPTURE_END [m] (8.12). */
    static final double L_SHORT = 0.10;
    /** Pose older than this is stale [s] (8.12). */
    static final double T_STALE = 0.10;
    /** Invalid or frozen pose longer than this gives FAILED_POSE [s] (8.12). */
    static final double POSE_INVALID_LIMIT = 1.0;
    /** Stale-pose speed gate [m/s] (8.12). */
    static final double STALE_POSE_SPEED_GATE = 0.3;

    /**
     * @param path      the active (flipped) path
     * @param profile   speed profile built from the measured start speed
     * @param heading   heading profile
     * @param law       the speed law
     * @param schedule  events of the active path
     * @param config    follower gains and tolerances
     * @param constraints resolved constraints (omegaMax, alphaMax, vMax)
     */
    Tracker(Path path, SpeedProfile profile, HeadingProfile heading, SpeedLaw law,
            EventSchedule schedule, PathFollowerConfig config, PathConstraints constraints) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /**
     * Resets the run (8.14 initialize): s = 0, (v_cmd, a_cmd) = (v0, 0), latches, state.
     *
     * @param pose     measured pose
     * @param measured measured field speeds
     * @param nowSec   time [s]
     */
    void start(Pose2d pose, ChassisSpeeds measured, double nowSec) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /**
     * One execute (8.14): projection, events, state transitions, translation and heading laws.
     *
     * @param pose     measured pose, blue-origin
     * @param measured measured field speeds
     * @param nowSec   time [s]
     * @return the field-relative command, omega CCW
     */
    ChassisSpeeds update(Pose2d pose, ChassisSpeeds measured, double nowSec) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /**
     * The finish and timeout test of isFinished (8.7.2, 8.7.3).
     *
     * @param nowSec time [s]
     * @return true when the run is over; {@link #result()} says why
     */
    boolean isFinished(double nowSec) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return the current result, RUNNING until it ends */
    FollowResult result() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return the current state */
    State state() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return progress s* [m] */
    double s() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return cross-track error e_n [m] */
    double eN() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return the speed law's v_cmd [m/s] */
    double vRef() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return heading error in degrees */
    double eThetaDeg() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return segment index at s* */
    int segment() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return the CAPTURE_END law on P_end, used by holdEndCommand (8.7.4) */
    ChassisSpeeds holdEnd(Pose2d pose, ChassisSpeeds measured) {
        throw new UnsupportedOperationException("TODO phase 3");
    }
}
