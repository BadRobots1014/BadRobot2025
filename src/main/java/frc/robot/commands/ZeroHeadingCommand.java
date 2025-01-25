package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCommand extends Command {

  private final SwerveSubsystem m_subsystem;
  private PS4Controller m_controller;

  public ZeroHeadingCommand(SwerveSubsystem subsystem, PS4Controller controller) {
    m_subsystem = subsystem;
    m_controller = controller;
  }

  @Override
  public void execute() {
    m_subsystem.resetPose();
    m_controller.setRumble(RumbleType.kBothRumble, 0.2);
  }

  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleType.kRightRumble, 0);
    m_controller.setRumble(RumbleType.kLeftRumble, 0);
  }
}
