// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;

public class ColorSensorSubsystem extends SubsystemBase {

  private ColorSensorV3 sensor;
  private final ShuffleboardTab m_tab;
  
  public ColorSensorSubsystem() {

    m_tab = Shuffleboard.getTab("Color sensor");
    m_tab.addNumber("R", this::getRed);
    m_tab.addNumber("G", this::getGreen);
    m_tab.addNumber("B", this::getBlue);
    m_tab.addBoolean("Is Connected", this::isConnected);

    sensor = new ColorSensorV3(Port.kMXP);
  }

  public Color getColor() {return sensor.getColor();}
  public double getRed() {return sensor.getRed();}
  public double getGreen() {return sensor.getGreen();}
  public double getBlue() {return sensor.getBlue();}

  public boolean isConnected() {
    return sensor.isConnected();
  }
}
