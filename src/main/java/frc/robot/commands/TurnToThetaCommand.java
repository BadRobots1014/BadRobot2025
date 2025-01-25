// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TurnToThetaCommand extends SwerveDriveCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  static double driveThetad;
  SwerveSubsystem m_subsystem;

  double currentTheta;
  Supplier<Double> targetTheta;

  PIDController turningPID;

  private static Supplier<Double> driveTheta = new Supplier<Double>() {
    @Override
    public Double get() {
      return driveThetad;
    }
  };
  
  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta)
  {
    super(
      swerveSubsystem,
      () -> 0d,
      () -> 0d,
      driveTheta,
      false,
      () -> false,
      () -> false,
      () -> -1d,
      () -> 0d,
      () -> 0d
    );
    this.targetTheta = targetTheta;
    m_subsystem = swerveSubsystem;

    turningPID = new PIDController(1.5, 0, 0);
  }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTheta = m_subsystem.getYaw();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double currentAngle = m_subsystem.getRotation2d().getDegrees();
    // double currentError = Math.abs(currentAngle - targetTheta);

    // //check which way is closer to target
    // boolean turnClockwise = ((currentAngle + 180) % 360) > targetTheta;

    currentTheta = m_subsystem.getRotation2d().getRadians();
    double error = targetTheta.get() - currentTheta;
    System.out.println("target" + targetTheta);
    System.out.println("current" + currentTheta);
    driveThetad = turningPID.calculate(error, 0);
    System.out.println("drive speed " + driveThetad);
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
