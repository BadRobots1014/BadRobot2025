// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

  private final SparkFlex topMotor;
  private final SparkFlex bottomMotor;

  /** Creates a new ExampleSubsystem. */
  public AlgaeSubsystem() {
    topMotor = new SparkFlex(AlgaeConstants.kLeftCanId, MotorType.kBrushless);
    bottomMotor = new SparkFlex(AlgaeConstants.kRightCanId, MotorType.kBrushless);

    SparkMaxConfig topMotorConfig = new SparkMaxConfig();
    SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();

    topMotorConfig.idleMode(IdleMode.kBrake);
    topMotorConfig.inverted(false);

    bottomMotorConfig.idleMode(IdleMode.kBrake);
    bottomMotorConfig.inverted(false);

    topMotor.configure(topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void SpinIn(double power) {
    SpinTop(power);
    SpinBottom(power);
  }

  public void SpinOut(double power) {
    SpinIn(-power);
  }

  public void SpinTop(double power) {
    topMotor.set(power * AlgaeConstants.kTopMaxSpeed);
  }

  public void SpinBottom(double power) {
    bottomMotor.set(power * AlgaeConstants.kBottomMaxSpeed);
  }

  public void Stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
