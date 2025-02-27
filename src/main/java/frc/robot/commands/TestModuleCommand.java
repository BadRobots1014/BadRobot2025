package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TestModuleCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  private final SwerveModuleState[] moduleStates;

  public TestModuleCommand(SwerveSubsystem subsystem, SwerveModuleState[] statesToTest) {
    swerveSubsystem = subsystem;
    moduleStates = statesToTest;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
