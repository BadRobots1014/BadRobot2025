package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  private ShuffleboardTab m_tab;
  private GenericEntry shuffleFieldOriented;

  private double xSpeed, ySpeed, turningSpeed;
  private boolean nudge;

  public SwerveDriveCommand(SwerveSubsystem subsystem) {
    swerveSubsystem = subsystem;
    
    addRequirements(swerveSubsystem);

    m_tab = Shuffleboard.getTab("Swerve Instance");
    shuffleFieldOriented = m_tab.add("Field Oriented" + this.toString(), DriveConstants.kFieldOriented).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  }

  @Override
  public void execute() {

    // Set chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (shuffleFieldOriented.getBoolean(DriveConstants.kFieldOriented) && !nudge) {
      // Field oriented if pov is not touched
      chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          turningSpeed,
          swerveSubsystem.getRotation2d()
        );
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // ChassisSpeeds to swerve module states
    SwerveModuleState[] moduleStates =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set module states
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void setDriveSpeeds(double xSpeed, double ySpeed, double turnSpeed, boolean nudge) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.turningSpeed = turnSpeed;
    this.nudge = nudge;
  }
}
