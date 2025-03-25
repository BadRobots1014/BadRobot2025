// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralMode;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CoralSubsystem m_subsystem;
  private CoralMode mode;

  private long startTime;

  public CoralCommand(CoralSubsystem subsystem, CoralMode mode) {
    m_subsystem = subsystem;
    this.mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setMotorMode(mode);
    startTime = System.nanoTime();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  // Calls end()
  @Override
  public boolean isFinished() {
    // End if over kCoralDurationSeconds
    // Nanosecond percision as it is theoretically better than milliseconds
    if (System.nanoTime() - startTime > CoralConstants.kCoralDurationNano && mode != CoralConstants.CoralMode.UP_OVERRIDE) {
      return true;
    } else {
      return false;
    }
  }
}