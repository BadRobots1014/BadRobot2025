// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final SparkMaxConfig limitedConfig;
  private final SparkMaxConfig unlimitedConfig;

  // Creates motor object and configures them
  public CoralSubsystem() {
    coralMotor = new SparkMax(CoralConstants.kCoralCanID, MotorType.kBrushed);

    // Default configuration for limits. Used during matches.
    limitedConfig = new SparkMaxConfig();
    limitedConfig.idleMode(IdleMode.kBrake);
    limitedConfig.limitSwitch.forwardLimitSwitchEnabled(true);

    // Configuration without the reverse (up) limit. Used in prematch setup.
    unlimitedConfig = limitedConfig;
    unlimitedConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Retrieves the amount of AMPs in the motor
  public double getMotorCurrent() {
    return coralMotor.getOutputCurrent();
  }

  public void setMotor(Double speed, boolean limited) {
    if (limited) {
      if (coralMotor.getReverseLimitSwitch() != null) {
        coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
      }
      coralMotor.set(speed);
    }
    else {
      if (coralMotor.getReverseLimitSwitch() != null) {
        coralMotor.configure(unlimitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
      }
      coralMotor.set(speed);
    }
    // if (mode == CoralMode.UP) {
    //   // Up
    //   if (coralMotor.getReverseLimitSwitch() == null) {
    //     coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
    //   }
    //   coralMotor.set(CoralConstants.kCoralUpSpeed);
    // }
    // else if (mode == CoralMode.DOWN) {
    //   // Down
    //   if (coralMotor.getReverseLimitSwitch() == null) {
    //     coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
    //   }
    //   coralMotor.set(CoralConstants.kCoralDownSpeed);
    // }
    // else {
    //   // Up without limit
    //   if (coralMotor.getReverseLimitSwitch() != null) {
    //     coralMotor.configure(unlimitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
    //   }
    //   coralMotor.set(CoralConstants.kCoralUpSpeed);
    // }
  }

  public void setMotorWithLimit(boolean up){
    if (up) {
      if (coralMotor.getReverseLimitSwitch() != null) {
        coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
      }
      coralMotor.set(CoralConstants.kCoralUpSpeed);
    }
    else {
      if (coralMotor.getReverseLimitSwitch() != null) {
        coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
      }
      coralMotor.set(CoralConstants.kCoralDownSpeed);
    }
    // if (mode == CoralMode.UP) {
    //   // Up
    //   if (coralMotor.getReverseLimitSwitch() == null) {
    //     coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
    //   }
    //   coralMotor.set(CoralConstants.kCoralUpSpeed);
    // }
    // else if (mode == CoralMode.DOWN) {
    //   // Down
    //   if (coralMotor.getReverseLimitSwitch() == null) {
    //     coralMotor.configure(limitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
    //   }
    //   coralMotor.set(CoralConstants.kCoralDownSpeed);
    // }
    // else {
    //   // Up without limit
    //   if (coralMotor.getReverseLimitSwitch() != null) {
    //     coralMotor.configure(unlimitedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // These changes are for mid match only and shouldn't persist
    //   }
    //   coralMotor.set(CoralConstants.kCoralUpSpeed);
    // }
  }

  // Stops the motor
  public void stopMotor() {
    coralMotor.stopMotor();
  }
}