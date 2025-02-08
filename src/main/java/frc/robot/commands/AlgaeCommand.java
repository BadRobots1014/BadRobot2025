// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeCommand extends Command {
  private final AlgaeSubsystem m_subsystem;
  private final boolean targetOpen;

  public AlgaeCommand(AlgaeSubsystem subsystem, boolean open) {
    m_subsystem = subsystem;

    targetOpen = open;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetOpen) {
      m_subsystem.OpenGrabber(AlgaeConstants.kPower);
    } else {
      m_subsystem.CloseGrabber(AlgaeConstants.kPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
