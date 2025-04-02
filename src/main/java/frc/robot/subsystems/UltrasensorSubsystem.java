// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.UltraSonicConstants;

public class UltrasensorSubsystem extends SubsystemBase {

  private static final AnalogInput sensor = new AnalogInput(UltraSonicConstants.kUltraSonicPort);

  public UltrasensorSubsystem() {}
  public static double getVoltage() {
    return sensor.getVoltage();
  }
  
  public static double getDistance() {
    return getVoltage() * UltraSonicConstants.kVoltsToDistance;
  }

  public static boolean isObjectDetected() {
    return getDistance() < UltraSonicConstants.kDistanceThreshold;
  }
}
