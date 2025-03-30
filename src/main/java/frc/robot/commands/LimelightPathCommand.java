package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
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
    if (limelightSubsystem == null) {
      currentCommand = swerveSubsystem.PathToLimelight(X, Y, Rot);
      currentCommand.initialize();
    }
    else if (LimelightHelpers.getFiducialID("") != -1){
      var lastPosLimelight = LimelightHelpers.getBotPose3d_TargetSpace("");
      currentCommand = swerveSubsystem.PathToLimelight(() -> lastPosLimelight.getY(), () -> lastPosLimelight.getZ() + DriveConstants.kAutoTargetDistance, () -> Rotation2d.fromRadians(lastPosLimelight.getRotation().getX()));
      currentCommand.initialize();
    }
  }

  @Override
  public void execute() {
    if (currentCommand != null) {
      currentCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (currentCommand != null) currentCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    if (currentCommand != null) return currentCommand.isFinished();
    else return true;
  }
}
