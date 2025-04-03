// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

public class DistanceSensorSubsystem extends SubsystemBase {

  private Rev2mDistanceSensor distSens;

  private static final String profileDefault = "Default";
  private static final String highSpeed = "High Speed";
  private static final String highAccuracy = "High Accuracy";
  private static final String longRange = "Long Range";
  private String m_profileSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final ShuffleboardTab m_tab;

  /** Creates a new ExampleSubsystem. */
  public DistanceSensorSubsystem() {
    
    m_chooser.setDefaultOption("High Speed", highSpeed);
    m_chooser.addOption("Default", profileDefault);
    m_chooser.addOption("High Accuracy", highAccuracy);
    m_chooser.addOption("Long Range", longRange);
    SmartDashboard.putData("Profile", m_chooser);
    m_tab = Shuffleboard.getTab("Distance sensor");
    m_tab.addNumber("Range", this::getRange);

    distSens = new Rev2mDistanceSensor(Port.kMXP);
    distSens.setAutomaticMode(true);
    distSens.setEnabled(true);

    distSens.setRangeProfile(RangeProfile.kHighSpeed);
  }

  public double getRange() {
    return distSens.GetRange();
  }
}
