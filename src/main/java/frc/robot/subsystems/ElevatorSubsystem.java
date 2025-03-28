// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Elastic;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftElevator;
  private final SparkMax rightElevator;
  private final AbsoluteEncoder encoder;
  private final RelativeEncoder relativeEncoder;

  private int rollover = 0;
  private double lastPos;

  private ShuffleboardTab m_tab;

  // Creates Left/Right climber objects and configures them
  public ElevatorSubsystem() {
    leftElevator = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
    rightElevator = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
    encoder = leftElevator.getAbsoluteEncoder();
    relativeEncoder = leftElevator.getEncoder();

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
    m_tab.addBoolean("Bottom limit", this::getReverseLimitSwitch);
    m_tab.addBoolean("Top Limit", this::getForwardLimitSwitch);
    m_tab.addNumber("Relative Encoder", this::getRelativeEncoder);

    resetRollover();
  }

  // Throughout the code we use left to get the values as the right is following the left so it should be the same.. I hope

  // Retrieves the amount of AMPs in the Left/Right climbers
  public double getElevatorCurrent() {
    return leftElevator.getOutputCurrent();
  }

  public double getElevatorEncoder()
  {
    double pos = encoder.getPosition();
    if (pos > 0.75 && lastPos < 0.25) {
      rollover++;
    }
    else if (pos < 0.25 && lastPos > 0.75) {
      rollover--;
    }
    lastPos = pos;
    return pos;
  }

  public double getRolloverAbsoluteEncoder() {
    return getElevatorEncoder() + rollover;
  }

  public double getRelativeEncoder() {
    return relativeEncoder.getPosition();
  }

  public void resetRollover() {
    rollover = 0;
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

  public boolean getReverseLimitSwitch() {
    return leftElevator.getReverseLimitSwitch().isPressed();
  }

  public boolean getForwardLimitSwitch() {
    return leftElevator.getForwardLimitSwitch().isPressed();
  }

  /*
  public double getElevatorEncoder() {
  return leftElevator.getAbsoluteEncoder().getPosition();
  }  
  */

}