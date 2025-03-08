// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CoralControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToCoralCommand extends Command {
  private final SwerveSubsystem m_subsystem;
  private final double m_direction;

  public AlignToCoralCommand(SwerveSubsystem swerveSubsystem, double direction) {
    m_subsystem = swerveSubsystem;
    m_direction = direction;
    addRequirements(swerveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, m_direction * 0.5, 0);

    // Divide and conquer
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Actually do the thing
    m_subsystem.setModuleStates(moduleStates);

    System.out.println(m_direction);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
