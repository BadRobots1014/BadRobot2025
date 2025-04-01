// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralMode;
import frc.robot.subsystems.CoralSubsystem;

import java.util.concurrent.BlockingDeque;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CoralSubsystem m_subsystem;
  private Supplier<Double> target;
  private double speed;
  private boolean withinDeadband = false;

  public CoralCommand(CoralSubsystem subsystem, Supplier<Double> target) {
    m_subsystem = subsystem;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public CoralCommand(CoralSubsystem subsystem, double speed) {
    this.speed = speed;
    m_subsystem = subsystem;
    target = null;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (target != null) {
      withinDeadband = m_subsystem.setMotorPreset(target.get());
    }
    else {
      m_subsystem.setMotor(speed);
    }
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
    return withinDeadband;
  }
}