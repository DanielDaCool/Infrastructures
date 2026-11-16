package frc.robot.Path.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Path.DemaciaTrajectory;

public class PathCommand extends Command {
  DemaciaTrajectory trajectory;
  Supplier<Pose2d> poseSupplier;
  Supplier<ChassisSpeeds> speedsSupplier; // field relative
  Consumer<ChassisSpeeds> speedsConsumer; // field relative

  public PathCommand(DemaciaTrajectory trajectory, Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier, Consumer<ChassisSpeeds> speedsConsumer, Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.speedsConsumer = speedsConsumer;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    trajectory.startTrajectory();
  }

  @Override
  public void execute() {
    speedsConsumer.accept(trajectory.calculateSpeeds(speedsSupplier.get(), poseSupplier.get()));
  }

  @Override
  public void end(boolean interrupted) {
    speedsConsumer.accept(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return trajectory.getIsFinishedTrajectory();
  }
}
