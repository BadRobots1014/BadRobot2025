package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightPathCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> X;
  private final Supplier<Double> Y;
  private final Supplier<Rotation2d> Rot;
  private Command currentCommand;

  public LimelightPathCommand(SwerveSubsystem subsystem, Supplier<Double> endX, Supplier<Double> endY, Supplier<Rotation2d> endRot) {
    swerveSubsystem = subsystem;
    X = endX;
    Y = endY;
    Rot = endRot;
    addRequirements(swerveSubsystem);
  }

  public LimelightPathCommand(SwerveSubsystem subsystem, LimelightSubsystem limelight) {
    addRequirements(swerveSubsystem);
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    currentCommand = swerveSubsystem.PathToLimelight(X, Y, Rot);
    currentCommand.initialize();
  }

  @Override
  public void execute() {
    currentCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    currentCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return currentCommand.isFinished();
  }
}
