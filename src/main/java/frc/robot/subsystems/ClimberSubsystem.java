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

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax leftClimber;
  private final SparkMax rightClimber;

  public ClimberSubsystem() {
    leftClimber = new SparkMax(ClimberConstants.kLeftClimberCanId, MotorType.kBrushless);
    rightClimber = new SparkMax(ClimberConstants.kRightClimberCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    leftClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getLeftClimberCurrent() {return leftClimber.getOutputCurrent();}
  public double getRightClimberCurrent() {return rightClimber.getOutputCurrent();}

  /**
   * Runs the climber at a speed
   * 
   * @param power The power to run the climber at
   * 
   */
  public void runClimber(double power) {
    rightClimber.set(getLeftClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
    leftClimber.set(getRightClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
  }

  /**
   * Stops the climbers
   * 
   */
  public void stopClimber() {
    rightClimber.stopMotor();
    leftClimber.stopMotor();
  }


}

