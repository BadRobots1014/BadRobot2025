package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCommand extends Command {

  private final SwerveSubsystem m_subsystem;
  private final Rotation2d angle;

  public ZeroHeadingCommand(SwerveSubsystem subsystem) {
    this(subsystem, new Rotation2d());
  }

  public ZeroHeadingCommand(SwerveSubsystem subsystem, Rotation2d newAngle) {
    m_subsystem = subsystem;
    angle = newAngle;
  }

  @Override
  public void execute() {
    m_subsystem.resetPose(new Pose2d(0, 0, angle));
    m_subsystem.resetOdometry();
  }
}