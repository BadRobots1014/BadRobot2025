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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase {

  //private final SparkMax leftClimber;
  //private final SparkMax rightClimber;
  private final SparkMax winch;


  // Creates Left/Right climber objects and configures them
  public WinchSubsystem() {
    //leftClimber = new SparkMax(ClimberConstants.kLeftClimberCanId, MotorType.kBrushless);
    //rightClimber = new SparkMax(ClimberConstants.kRightClimberCanId, MotorType.kBrushless);
    winch = new SparkMax(WinchConstants.kWinchCanId, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(80);

    //leftClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //rightClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    winch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the climber at a certain power level (speed)
   * @param power The power in AMPs to run the climber at
   */
  public void runWinch(double power) {
    winch.set(-power);
  }

  // Stops the climber motors
  public void stopWinch() {

    winch.stopMotor();
  }


}
