package frc.robot.Path;

/** Why a {@link FollowPathCommand} ended (design doc section 9.10). */
public enum FollowResult {
    /** Still running (or not started). */
    RUNNING,
    /** Position and angle in tolerance for t_settle. */
    OK,
    /** t_timeout passed; zero speed sent once. */
    FAILED_TIMEOUT,
    /** More than startTol from P0 at start. */
    FAILED_START_TOO_FAR,
    /** Pose NaN or stale for more than 1 s. */
    FAILED_POSE,
    /** Cancelled by the scheduler or the driver. */
    INTERRUPTED;

    /** @return true if this is {@link #OK} */
    public boolean isSuccess() {
        return this == OK;
    }

    /** @return true for the FAILED_* results */
    public boolean isFailure() {
        return this == FAILED_TIMEOUT || this == FAILED_START_TOO_FAR || this == FAILED_POSE;
    }
}
