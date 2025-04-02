// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.UltraSonicConstants;

public class UltrasensorSubsystem extends SubsystemBase {

  private static final AnalogInput sensor = new AnalogInput(UltraSonicConstants.kUltraSonicPort);
  private ShuffleboardTab m_tab;

  public UltrasensorSubsystem() {
    m_tab = Shuffleboard.getTab("Ultrasonic Sensor");
    m_tab.add(sensor);
  }

  public static double getVoltage() {
    double voltageScaleFactor = 5/RobotController.getVoltage5V();
    return sensor.getVoltage() * voltageScaleFactor;
  }
  
  public static double getDistance() {
    return getVoltage() * UltraSonicConstants.kVoltsToDistance;
  }

  public static boolean isObjectDetected() {
    return getDistance() < UltraSonicConstants.kDistanceThreshold;
  }
}
