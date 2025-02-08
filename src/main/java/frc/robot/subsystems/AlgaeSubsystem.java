// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  /** Creates a new ExampleSubsystem. */
  public AlgaeSubsystem() {
    leftMotor = new SparkMax(AlgaeConstants.kLeftCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(AlgaeConstants.kRightCanId, MotorType.kBrushless);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.idleMode(IdleMode.kBrake);
    // leftMotorConfig.inverted(false);

    rightMotorConfig.idleMode(IdleMode.kBrake);
    // rightMotorConfig.inverted(true);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getMotorCurrent() {
    return leftMotor.getOutputCurrent();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void OpenGrabber(double power) {
    leftMotor.set(getMotorCurrent() < AlgaeConstants.kMaxAmps ? power : 0);
    rightMotor.set(getMotorCurrent() < AlgaeConstants.kMaxAmps ? power : 0);
  }

  public void CloseGrabber(double power) {
    OpenGrabber(-power);
  }

  public void Stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
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
