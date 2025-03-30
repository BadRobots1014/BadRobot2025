package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.WinchSubsystem;

public class WinchCommand extends Command {
  private final WinchSubsystem m_subsystem;
  private final double power;

  public WinchCommand(WinchSubsystem subsystem, double power) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    this.power = power;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runWinch(power); // Runs both climbers at the same time, so there is no need to run it twice. I am not sure how we should reference the power from both
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}