package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class NudgeToReefCommand extends SwerveDriveCommand {

  private final DistanceSensorSubsystem subsystem;
  private final double range;

  public NudgeToReefCommand(SwerveSubsystem swerveSubsystem, DistanceSensorSubsystem sensorSubsystem, Supplier<Double> pov, double range) {
    super(
      swerveSubsystem,
      () -> 0d,
      () -> 0d,
      () -> 0d,
      false,
      () -> false,
      () -> false,
      pov,
      () -> 0d,
      () -> 0d
    );

    subsystem = sensorSubsystem;
    this.range = range;
  }

  @Override
  public boolean isFinished() {
    return subsystem.getRange() < range;
  }
}
