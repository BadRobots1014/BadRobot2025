// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class BlinkinSubsystem extends SubsystemBase {

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Blinkin");

  private final Spark blinkin;

  public boolean party = false;

  public BlinkinSubsystem() {
    blinkin = new Spark(BlinkinConstants.kBlinkinPort);
    m_tab.add(blinkin.toString(), blinkin);
    setDefault();
  }

  /**
   * Sets the LED pattern for the REV Blinkin LED Driver.
   *
   * The pattern code determines the color or animation displayed by the LEDs.
   * A full list of pattern codes and their meanings can be found in the REV
   * Blinkin LED Driver User Manual:
   * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14
   *
   * @param pattern A double value representing the desired LED pattern code.
   *
   */
  public void setBlinkin(double pattern) {
    if (party) {
      blinkin.set(BlinkinConstants.colorCode.get("rainbow-rainbow"));
    }
    blinkin.set(pattern);
  }

  public void setBlinkin(String pattern) {
    setBlinkin(BlinkinConstants.colorCode.get(pattern));
  }

  public void setDefault(){
    setBlinkin(BlinkinConstants.kBlinkinDefaultColorCode);
  }

}
