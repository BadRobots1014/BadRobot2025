package frc.robot.commands;
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
    return subsystem.getRange() < range && subsystem.getRange() != -1;
  }
}
