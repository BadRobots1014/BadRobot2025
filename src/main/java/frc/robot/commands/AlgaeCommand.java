// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeCommand extends Command {
  private final AlgaeSubsystem m_subsystem;
  private final boolean spinIn;

  public AlgaeCommand(AlgaeSubsystem subsystem, boolean spinIn) {
    m_subsystem = subsystem;

    this.spinIn = spinIn;
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
    if (spinIn) {
      m_subsystem.SpinIn(AlgaeConstants.kPower);
    } else {
      m_subsystem.SpinOut(AlgaeConstants.kPower);
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
