// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class ElevatorCommandWithEnd extends ElevatorCommand {
  
  /**
   * @param elevatorsubsystem The subsystem referenced in this command
   * @param targetLevel       A number 1-4 in which you want to set the height of
   *                          the elevator to
   */
  public ElevatorCommandWithEnd(ElevatorSubsystem elevatorsubsystem, Supplier<Double> targetLevel) {
    super(elevatorsubsystem, targetLevel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(super.goalLevelSupplier.get() - super.m_subsystem.getRolloverAbsoluteEncoder()) <= ElevatorConstants.kElevatorDeadband;
  }
}
