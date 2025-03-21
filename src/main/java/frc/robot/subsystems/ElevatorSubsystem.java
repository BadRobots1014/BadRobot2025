// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Elastic;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftElevator;
  private final SparkMax rightElevator;
  private final SparkMax fakeMotor;
  private final AbsoluteEncoder encoder;

  private ShuffleboardTab m_tab;

  // Creates Left/Right climber objects and configures them
  public ElevatorSubsystem() {
    leftElevator = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
    rightElevator = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
    fakeMotor = new SparkMax(ElevatorConstants.kEncoderCanId, MotorType.kBrushless);
    encoder = fakeMotor.getAbsoluteEncoder();

    SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();
    SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();

    leftElevatorConfig.idleMode(IdleMode.kBrake);
    leftElevatorConfig.inverted(false);

    rightElevatorConfig.idleMode(IdleMode.kBrake);
    rightElevatorConfig.follow(ElevatorConstants.kLeftElevatorCanId, true);

    leftElevator.configure(leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_tab = Shuffleboard.getTab("Elevator");
    m_tab.addNumber("Encoder", this::getElevatorEncoder);
  }

  // Throughout the code we use left to get the values as the right is following the left so it should be the same.. I hope

  // Retrieves the amount of AMPs in the Left/Right climbers
  public double getElevatorCurrent() {
    return leftElevator.getOutputCurrent();
  }

  public double getElevatorEncoder()
  {
    return encoder.getPosition();
  }

  /**
   * Runs the elevator at a certain speed determined by the amount of AMPs
   * 
   * @param power The power in AMPs to run the climber at. +Power = Up, -Power =
   *              Down
   */
  public void runElevator(double power) {
    System.out.println("Setting power to " + power);
    leftElevator.set(getElevatorCurrent() < ElevatorConstants.kElevatorMaxAmps ? power : 0);
  }

  // Stops the climber motors
  public void stopElevator() {
    leftElevator.stopMotor();
  }

  /*
  public double getElevatorEncoder() {
  return leftElevator.getAbsoluteEncoder().getPosition();
  }  
  */

}