package frc.robot.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Drives a blue-side {@link Path}, flipped on the red alliance. Ends at once if the robot is not near the start. */
public class FollowPathCommand extends Command {
    private final Path path;
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> fieldSpeeds;
    private final Consumer<ChassisSpeeds> driveFieldRelative;
    private Path active;
    private PathFollower follower; // null = not started or start too far

    /**
     * @param pose field pose [m, rad]
     * @param fieldSpeeds measured field-relative speeds [m/s]
     * @param driveFieldRelative takes field-relative speeds [m/s, rad/s]
     */
    public FollowPathCommand(Path path, Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds,
            Consumer<ChassisSpeeds> driveFieldRelative, Subsystem drive) {
        this.path = path;
        this.pose = pose;
        this.fieldSpeeds = fieldSpeeds;
        this.driveFieldRelative = driveFieldRelative;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        active = red ? path.flipped() : path;
        Pose2d start = pose.get();
        double distance = start.getTranslation().getDistance(active.start());
        follower = null;
        if (distance > PathConstants.START_TOLERANCE) {
            DriverStation.reportError(String.format("FollowPathCommand: robot %.2f m from path start", distance), false);
        } else {
            follower = new PathFollower(active, start);
        }
    }

    @Override
    public void execute() {
        if (follower != null) driveFieldRelative.accept(follower.calculate(pose.get(), fieldSpeeds.get()));
    }

    @Override
    public boolean isFinished() { return follower == null || follower.isFinished(); }

    @Override
    public void end(boolean interrupted) { driveFieldRelative.accept(new ChassisSpeeds()); }

    /** Fraction of the path driven, 0..1. */
    public double progress() { return follower == null ? 0 : follower.progress() / active.length(); }
}
