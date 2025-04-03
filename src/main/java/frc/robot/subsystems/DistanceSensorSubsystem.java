// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

public class DistanceSensorSubsystem extends SubsystemBase {

  private Rev2mDistanceSensor distSens;
  private final SendableChooser<RangeProfile> m_chooser = new SendableChooser<>();
  private final ShuffleboardTab m_tab;
  private RangeProfile lastProfile;
  
  public DistanceSensorSubsystem() {
    
    m_chooser.setDefaultOption("High Speed", RangeProfile.kHighSpeed);
    m_chooser.addOption("Default", RangeProfile.kDefault);
    m_chooser.addOption("High Accuracy", RangeProfile.kHighAccuracy);
    m_chooser.addOption("Long Range", RangeProfile.kLongRange);
    SmartDashboard.putData("Profile", m_chooser);
    m_tab = Shuffleboard.getTab("Distance sensor");
    m_tab.addNumber("Range", this::getRange);
    m_tab.addBoolean("Is Range Valid", this::isRangeValid);
    m_tab.addBoolean("Is Enabled", this::isEnabled);

    distSens = new Rev2mDistanceSensor(Port.kMXP);
    distSens.setAutomaticMode(true);
    distSens.setEnabled(true);

    updateProfile();
  }

  public double getRange() {
    if (lastProfile != m_chooser.getSelected()) {
      distSens.setRangeProfile(m_chooser.getSelected());
    }
    lastProfile = m_chooser.getSelected();
    return distSens.GetRange();
  }

  public boolean isRangeValid() {
    return distSens.isRangeValid();
  }

  public boolean isEnabled() {
    return distSens.isEnabled();
  }

  public void updateProfile() {
    distSens.setRangeProfile(m_chooser.getSelected());
  }
}
