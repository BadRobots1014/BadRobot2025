// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeCommand extends Command {
  private final AlgaeSubsystem m_subsystem;
  private final boolean spinIn;
  private final Supplier<Double> leftSpdFunction, rightSpdFunction;

  public AlgaeCommand(AlgaeSubsystem subsystem, boolean spinIn) {
    m_subsystem = subsystem;

    this.spinIn = spinIn;
    leftSpdFunction = () -> 0d;
    rightSpdFunction = () -> 0d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public AlgaeCommand(AlgaeSubsystem subsystem, Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier) {
    m_subsystem = subsystem;

    leftSpdFunction = leftYSupplier;
    rightSpdFunction = rightYSupplier;

    this.spinIn = false;

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
    // Uncomment when set power values are decided
    // if (spinIn) {
    //   m_subsystem.SpinIn(AlgaeConstants.kPower);
    // } else {
    //   m_subsystem.SpinOut(AlgaeConstants.kPower);
    // }
    m_subsystem.SpinTop(leftSpdFunction.get());
    m_subsystem.SpinBottom(rightSpdFunction.get());
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
