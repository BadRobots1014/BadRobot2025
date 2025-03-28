// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {

  private final ElevatorSubsystem m_subsystem;
  private final Supplier<Double> goalLevelSupplier;
  private final Supplier<Double> goalSpeedSupplier;
  private PIDController m_pidController;
  
  /**
   * @param elevatorsubsystem The subsystem referenced in this command
   * @param targetLevel       A number 1-4 in which you want to set the height of
   *                          the elevator to
   */
  public ElevatorCommand(ElevatorSubsystem elevatorsubsystem, Supplier<Double> targetLevel) {
    m_subsystem = elevatorsubsystem;
    goalLevelSupplier = targetLevel;
    goalSpeedSupplier = null;

    // Uses the same PID controller so they should be synched. Change variables in
    // Constants file
    m_pidController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
        ElevatorConstants.kElevatorP);

    // Ensures that the runElevator and stopElevator functions are present
    addRequirements(elevatorsubsystem);

    System.out.println("Elevator Command configured");

  }

  public ElevatorCommand(ElevatorSubsystem elevatorsubsystem, Supplier<Double> speed, boolean eh) {
    m_subsystem = elevatorsubsystem;
    goalSpeedSupplier = speed;
    goalLevelSupplier = null;

    // Uses the same PID controller so they should be synched. Change variables in
    // Constants file
    m_pidController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorP);

    // Ensures that the runElevator and stopElevator functions are present

    System.out.println("Elevator Command configured (womp womp)");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Elevator started with goal level: " + goalLevelSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goalLevelSupplier != null) {
      if (Math.abs(goalLevelSupplier.get() - m_subsystem.getElevatorEncoder()) >= 0.1) {
        m_subsystem.runElevator(m_pidController.calculate(m_subsystem.getElevatorEncoder(), goalLevelSupplier.get()));
      }
    }
    else {
      m_subsystem.runElevator(goalSpeedSupplier.get());
      System.out.print("aaaaaaaaaaaaaaa");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator done" + (interrupted ? " (interrupted)" : ""));
    // Just in case...
    m_subsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * // Not sure how exactly to calculate this...
     * return m_subsystem.getLeftElevatorEncoder() == goalPosition &&
     * m_subsystem.getLeftElevatorCurrent() == goalPosition;
     */
    return false;
  }
}
