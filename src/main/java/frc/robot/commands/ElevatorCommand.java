// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ElevatorSubsystem m_subsystem;
  private Integer m_targetLevel;

  /**
   * @param elevatorsubsystem The subsystem referenced in this command
   * @param targetLevel A number 1-4 in which you want to set the height of the elevator to
   */
  public ElevatorCommand(ElevatorSubsystem elevatorsubsystem, int targetLevel) {
    m_subsystem = elevatorsubsystem;
    m_targetLevel = targetLevel;
    addRequirements(elevatorsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Would somehow retrieve what level you are currently on and adjust itself using the runElevator/stopElevator functions
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
