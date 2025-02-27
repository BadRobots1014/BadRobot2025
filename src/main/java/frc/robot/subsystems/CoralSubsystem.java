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
import frc.robot.Constants.CoralMode;

public class CoralSubsystem extends SubsystemBase {

  private final SparkMax coralMotor;

  // Creates motor object and configures them
  public CoralSubsystem() {
    coralMotor = new SparkMax(1984, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);

    coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Retrieves the amount of AMPs in the motor
  public double getMotorCurrent() {
    return coralMotor.getOutputCurrent();
  }

  public void setMotorMode(CoralMode mode) {
    if (mode == CoralMode.In) {
      coralMotor.set(CoralConstants.kCoralInSpeed);
    } else {
      coralMotor.set(CoralConstants.kCoralInSpeed);
    }
  }

  // Stops the motor
  public void stopMotor() {
    coralMotor.stopMotor();
  }

}