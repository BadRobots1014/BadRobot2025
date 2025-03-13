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
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

  private final SparkMax coralMotor1;
  private final SparkMax coralMotor2;

  // Creates motor object and configures them
  public CoralSubsystem() {
    coralMotor1 = new SparkMax(CoralConstants.kCoralCanId1, MotorType.kBrushed);
    coralMotor2 = new SparkMax(CoralConstants.kCoralCanId2, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    coralMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Retrieves the amount of AMPs in motor 1
  public double getMotor1Current() {
    return coralMotor1.getOutputCurrent();
  }

  // Retrieves the amount of AMPs in motor 2
  public double getMotor2Current() {
    return coralMotor1.getOutputCurrent();
  }

  public void setMotorMode(boolean dump) {
    if (dump) {
      // Down
      coralMotor1.set(CoralConstants.kCoralOutSpeed);
      coralMotor2.set(CoralConstants.kCoralOutSpeed);
    } else {
      // Up
      coralMotor1.set(CoralConstants.kCoralInSpeed);
      coralMotor2.set(CoralConstants.kCoralInSpeed);
    }
  }

  // Stops the motor
  public void stopMotor() {
    coralMotor1.stopMotor();
    coralMotor2.stopMotor();
  }

}