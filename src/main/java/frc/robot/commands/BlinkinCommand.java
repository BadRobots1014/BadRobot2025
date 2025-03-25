// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.BlinkinConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class BlinkinCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final BlinkinSubsystem m_blinkinSubsystem;
  private final double m_pattern;

  /**
   * Sets blinkin to a color
   *
   * @param blinkinSubsystem The blinkin subsystem used by this command.
   * @param pattern A double value representing the desired LED pattern code. 
   *                Color values found here: 
   *                https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14
   * 
   */
  public BlinkinCommand(BlinkinSubsystem blinkinSubsystem, double pattern) {
    m_blinkinSubsystem = blinkinSubsystem;
    m_pattern = pattern;
    addRequirements(blinkinSubsystem);
  }

  public BlinkinCommand(BlinkinSubsystem blinkinSubsystem, String pattern) {
    this(blinkinSubsystem, BlinkinConstants.colorCode.get(pattern));
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_blinkinSubsystem.setBlinkin(m_pattern);
  }

  @Override
  public void end(boolean interrupted) {
    m_blinkinSubsystem.setDefault();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
