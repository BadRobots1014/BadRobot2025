// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.UltraSonicConstants;

public class UltrasensorSubsystem extends SubsystemBase {

  private static final AnalogInput sensor = new AnalogInput(UltraSonicConstants.kUltraSonicPort);

  /** Creates a new ExampleSubsystem. */
  public UltrasensorSubsystem() {}
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public static double getVoltage() {
    return sensor.getVoltage();
  }
  
  public static double getDistance() {
    return getVoltage() * UltraSonicConstants.kVoltsToDistance;
  }

  public static boolean isObjectDetected() {
    return getDistance() < UltraSonicConstants.kDistanceThreshold;
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
