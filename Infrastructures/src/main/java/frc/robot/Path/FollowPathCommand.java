package frc.robot.Path;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Kinematics.KinematicsConfig;

/**
 * Drives a {@link Path} (design doc section 9.10). A command, not a value: the {@code withX}
 * methods configure this object before it is scheduled and return it.
 */
public class FollowPathCommand extends Command {

    /**
     * @param path                the path (blue frame)
     * @param pose                blue-origin field pose
     * @param measuredFieldSpeeds field-relative measured speeds, omega CCW
     * @param output              receives the field-relative command, omega CCW
     * @param requirements        the drivetrain (at least one, V24)
     */
    public FollowPathCommand(Path path,
                             Supplier<Pose2d> pose,
                             Supplier<ChassisSpeeds> measuredFieldSpeeds,
                             Consumer<ChassisSpeeds> output,
                             Subsystem... requirements) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return this; motor curve, wheel cap, limit warning C2 */
    public FollowPathCommand withKinematics(KinematicsConfig config) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return this; R_mod for the wheel-speed cap (moduleRadius 0 = unknown) */
    public FollowPathCommand withPathsConfig(PathsConfig config) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return this; gains, tolerances, timeout, flip (default {@code PathFollowerConfig.defaults()}) */
    public FollowPathCommand withConfig(PathFollowerConfig config) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return this; called once with the result when the command ends */
    public FollowPathCommand withOnFinish(Consumer<FollowResult> callback) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    @Override
    public void initialize() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    @Override
    public void execute() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    @Override
    public boolean isFinished() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    @Override
    public void end(boolean interrupted) {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return why the command ended, RUNNING until then */
    public FollowResult getResult() {
        throw new UnsupportedOperationException("TODO phase 3");
    }

    /** @return a command holding P_end with the CAPTURE_END law until interrupted (8.7.4) */
    public Command holdEndCommand() {
        throw new UnsupportedOperationException("TODO phase 3");
    }
}
