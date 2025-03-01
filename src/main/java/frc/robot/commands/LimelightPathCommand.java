package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

public class LimelightPathCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final LimelightSubsystem limelightSubsystem;
  private final Supplier<Double> X;
  private final Supplier<Double> Y;
  private final Supplier<Rotation2d> Rot;
  private Command currentCommand;

  public LimelightPathCommand(SwerveSubsystem subsystem, Supplier<Double> endX, Supplier<Double> endY, Supplier<Rotation2d> endRot) {
    swerveSubsystem = subsystem;
    limelightSubsystem = null;
    X = endX;
    Y = endY;
    Rot = endRot;
    addRequirements(swerveSubsystem);
  }

  public LimelightPathCommand(SwerveSubsystem subsystem, LimelightSubsystem limelight) {
    swerveSubsystem = subsystem;
    limelightSubsystem = limelight;
    X = () -> 1d;
    Y = () -> 0d;
    Rot = () -> new Rotation2d();
    addRequirements(swerveSubsystem);
    addRequirements(limelightSubsystem);
  }

  @Override
  public void initialize() {
    if (limelightSubsystem == null || LimelightHelpers.getFiducialID("") == -1) {
      currentCommand = swerveSubsystem.PathToLimelight(X, Y, Rot);
    }
    else {
      var lastPosLimelight = LimelightHelpers.getBotPose3d_TargetSpace("");
      currentCommand = swerveSubsystem.PathToLimelight(() -> lastPosLimelight.getZ(), () -> lastPosLimelight.getX(), () -> Rotation2d.fromRadians(lastPosLimelight.getRotation().getY()));
    }
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
