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
  private final Supplier<Integer> goalLevelSupplier;
  private PIDController m_pidController;

  private double goalPosition;

  /**
   * @param elevatorsubsystem The subsystem referenced in this command
   * @param targetLevel       A number 1-4 in which you want to set the height of
   *                          the elevator to
   */
  public ElevatorCommand(ElevatorSubsystem elevatorsubsystem, Supplier<Integer> targetLevel) {
    m_subsystem = elevatorsubsystem;
    goalLevelSupplier = targetLevel;

    // Uses the same PID controller so they should be synched. Change variables in
    // Constants file
    m_pidController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
        ElevatorConstants.kElevatorP);

    // Ensures that the runElevator and stopElevator functions are present
    addRequirements(elevatorsubsystem);

    System.out.println("Elevator Command configured");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator started with goal level: " + goalLevelSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // use constants for power later
    if (goalLevelSupplier.get() == 1)
      m_subsystem.runElevator(.9);
    if (goalLevelSupplier.get() == 2)
      m_subsystem.runElevator(-.9);

    // This code is good but commented out for now as we don't have encoders yet and not really that far into the elevator
    /*
     * switch(goalLevelSupplier.get()) {
     * 
     * // Sends PID controller calculations to both motors. runElevator essentially
     * replaces motor.set function
     * case 1:
     * m_subsystem.runElevator(m_pidController.calculate(m_subsystem.
     * getLeftElevatorEncoder(), ElevatorConstants.kLvlOnePos));
     * goalPosition = ElevatorConstants.kLvlOnePos;
     * break;
     * case 2:
     * m_subsystem.runElevator(m_pidController.calculate(m_subsystem.
     * getLeftElevatorEncoder(), ElevatorConstants.kLvlTwoPos));
     * goalPosition = ElevatorConstants.kLvlTwoPos;
     * break;
     * case 3:
     * m_subsystem.runElevator(m_pidController.calculate(m_subsystem.
     * getLeftElevatorEncoder(), ElevatorConstants.kLvlThreePos));
     * goalPosition = ElevatorConstants.kLvlThreePos;
     * break;
     * case 4:
     * m_subsystem.runElevator(m_pidController.calculate(m_subsystem.
     * getLeftElevatorEncoder(), ElevatorConstants.kLvlFourPos));
     * goalPosition = ElevatorConstants.kLvlFourPos;
     * break;
     * }
     */
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
   // Not sure how exactly to calculate this...
    return m_subsystem.getLeftElevatorEncoder() == goalPosition &&
    m_subsystem.getLeftElevatorCurrent() == goalPosition;
    */
    return false;
   }
}
