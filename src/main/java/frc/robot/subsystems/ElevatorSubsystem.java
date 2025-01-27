// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftElevator;
  private final SparkMax rightElevator;


  // Creates Left/Right climber objects and configures them
  public ElevatorSubsystem() {
    leftElevator = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
    rightElevator = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.idleMode(IdleMode.kBrake);

    leftElevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Retrieves the amount of AMPs in the Left/Right climbers
  public double getLeftElevatorCurrent() {return leftElevator.getOutputCurrent();}
  public double getRightElevatorCurrent() {return rightElevator.getOutputCurrent();}

  /**
   * Runs the elevator at a certain speed determined by the amount of AMPs
   * @param power The power in AMPs to run the climber at. +Power = Up, -Power = Down
   */
  public void runElevator(double power) {
    rightElevator.set(getRightElevatorCurrent() < ElevatorConstants.kElevatorMaxAmps ? power : 0);
    leftElevator.set(getLeftElevatorCurrent() < ElevatorConstants.kElevatorMaxAmps ? power : 0);
  }

  // Stops the climber motors
  public void stopElevator() {
    rightElevator.stopMotor();
    leftElevator.stopMotor();
  }


}