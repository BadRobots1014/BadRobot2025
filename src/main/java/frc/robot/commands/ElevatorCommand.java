// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {

  private final ElevatorSubsystem m_subsystem;
  private final Integer goalLevel;
  private PIDController m_pidController;

  private final SparkMax m_leftElevator;
  private final SparkMax m_rightElevator;

  private SparkAbsoluteEncoder leftEncoder;
  private SparkAbsoluteEncoder rightEncoder;

  private double goalPosition;
  private final double errorMargin = 0.1;

  /**
   * @param elevatorsubsystem The subsystem referenced in this command
   * @param targetLevel A number 1-4 in which you want to set the height of the elevator to
   */
  public ElevatorCommand(ElevatorSubsystem elevatorsubsystem, int targetLevel) {
    m_subsystem = elevatorsubsystem;
    goalLevel = targetLevel;

    m_pidController = new PIDController(1, 0, 0);

    m_leftElevator = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
    m_rightElevator = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.idleMode(IdleMode.kBrake);

    m_leftElevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = m_leftElevator.getAbsoluteEncoder();
    rightEncoder = m_rightElevator.getAbsoluteEncoder();


    addRequirements(elevatorsubsystem);

    System.out.println("Elevator Command configured");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator started with goal level: " + goalLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(goalLevel) {

      case 1:
        m_leftElevator.set(m_pidController.calculate(leftEncoder.getPosition(), ElevatorConstants.kLvlOnePos));
        m_rightElevator.set(m_pidController.calculate(rightEncoder.getPosition(), ElevatorConstants.kLvlOnePos));
        goalPosition = ElevatorConstants.kLvlOnePos;
        break;
      case 2:
        m_leftElevator.set(m_pidController.calculate(leftEncoder.getPosition(), ElevatorConstants.kLvlTwoPos));
        m_rightElevator.set(m_pidController.calculate(rightEncoder.getPosition(), ElevatorConstants.kLvlTwoPos));
        goalPosition = ElevatorConstants.kLvlTwoPos;
        break;
      case 3:
        m_leftElevator.set(m_pidController.calculate(leftEncoder.getPosition(), ElevatorConstants.kLvlThreePos));
        m_rightElevator.set(m_pidController.calculate(rightEncoder.getPosition(), ElevatorConstants.kLvlThreePos));
        goalPosition = ElevatorConstants.kLvlThreePos;
        break;
      case 4:
        m_leftElevator.set(m_pidController.calculate(leftEncoder.getPosition(), ElevatorConstants.kLvlFourPos));
        m_rightElevator.set(m_pidController.calculate(rightEncoder.getPosition(), ElevatorConstants.kLvlFourPos));
        goalPosition = ElevatorConstants.kLvlFourPos;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator done" + (interrupted ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(leftEncoder.getPosition() - goalPosition) < errorMargin && Math.abs(rightEncoder.getPosition() - goalPosition) < errorMargin;
  }
}
