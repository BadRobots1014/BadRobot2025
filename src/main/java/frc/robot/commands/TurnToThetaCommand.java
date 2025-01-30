package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.TurnThetaHelper;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToThetaCommand extends SwerveDriveCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  SwerveSubsystem m_subsystem;

  Supplier<Double> targetTheta;

  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta, Supplier<Double> moveX,
      Supplier<Double> moveY) {
    super(
        swerveSubsystem,
        moveX,
        moveY,
        swerveSubsystem.thetaHelper.driveTheta,
        false,
        () -> false,
        () -> false,
        () -> -1d,
        () -> 0d,
        () -> 0d);
    m_subsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Since the supplier will always point to the target value, it can be set
    // prematurely
    swerveSubsystem.thetaHelper.setTargetTheta(targetTheta);
  }

  @Override
  public void execute() {
    // double currentAngle = m_subsystem.getRotation2d().getDegrees();
    // double currentError = Math.abs(currentAngle - targetTheta);

    // //check which way is closer to target
    // boolean turnClockwise = ((currentAngle + 180) % 360) > targetTheta;

    // Begin rotating to target theta
    Rotation2d currentTheta = m_subsystem.getRotation2d();
    m_subsystem.thetaHelper.calculate(currentTheta);

    System.out.println("target" + targetTheta);
    System.out.println("current" + currentTheta);
    System.out.println("drive speed " + m_subsystem.thetaHelper.driveTheta.get());

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

// Noiritgc was here