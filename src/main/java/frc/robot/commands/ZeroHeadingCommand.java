package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCommand extends Command {

  private final SwerveSubsystem m_subsystem;

  public ZeroHeadingCommand(SwerveSubsystem subsystem) {
    m_subsystem = subsystem;
  }

  @Override
  public void execute() {
    m_subsystem.resetPose();
  }
}
