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

  //private final SparkMax leftClimber;
  //private final SparkMax rightClimber;
  private final SparkMax climber;


  // Creates Left/Right climber objects and configures them
  public ClimberSubsystem() {
    //leftClimber = new SparkMax(ClimberConstants.kLeftClimberCanId, MotorType.kBrushless);
    //rightClimber = new SparkMax(ClimberConstants.kRightClimberCanId, MotorType.kBrushless);
    climber = new SparkMax(ClimberConstants.kClimberCanId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    //leftClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //rightClimber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Retrieves the amount of AMPs in the Left/Right climbers
  //public double getLeftClimberCurrent() {return leftClimber.getOutputCurrent();}
  //public double getRightClimberCurrent() {return rightClimber.getOutputCurrent();}
  public double getClimberCurrent() {return climber.getOutputCurrent();}

  /**
   * Runs the climber at a certain power level (speed)
   * @param power The power in AMPs to run the climber at
   */
  public void runClimber(double power) {
    //rightClimber.set(getLeftClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
    //leftClimber.set(getRightClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
    climber.set(getClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
  }

  // Stops the climber motors
  public void stopClimber() {
    //rightClimber.stopMotor();
    //leftClimber.stopMotor();
    climber.stopMotor();
  }


}
