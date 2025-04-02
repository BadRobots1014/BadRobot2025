// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralMode;

public class CoralSubsystem extends SubsystemBase {

  private final SparkMax coralMotor;
  private final SparkMaxConfig coralConfig;
  private final AbsoluteEncoder encoder;
  // Creates motor object and configures them
  public CoralSubsystem() {
    coralMotor = new SparkMax(CoralConstants.kCoralCanID, MotorType.kBrushed);

    // Default configuration for limits. Used during matches.
    coralConfig = new SparkMaxConfig();
    coralConfig.idleMode(IdleMode.kBrake);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = coralMotor.getAbsoluteEncoder();
    }

  // Retrieves the amount of AMPs in the motor
  public double getMotorCurrent() {
    return coralMotor.getOutputCurrent();
  }

  public boolean setMotorPreset(double target) {
    double currentPos = encoder.getPosition();
    double displacement = currentPos - target;

    double speed = Math.abs(displacement) > CoralConstants.kCoralDeadband ? Math.copySign(CoralConstants.kCoralSpeed, displacement) : 0;

    coralMotor.set(speed);

    // Returns true if the position is within deadband
    return Math.abs(displacement) < CoralConstants.kCoralDeadband;
  }

  public void setMotor(double speed) {
    coralMotor.set(speed);
  }

  // Stops the motor
  public void stopMotor() {
    coralMotor.stopMotor();
  }
}