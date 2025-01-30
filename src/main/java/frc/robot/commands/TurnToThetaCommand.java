package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToThetaCommand extends SwerveDriveCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

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

  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta) {
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
        () -> 0d);
    this.targetTheta = targetTheta;
    m_subsystem = swerveSubsystem;

    turningPID = new PIDController(1, 0, 0);
    turningPID.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize() {
    currentTheta = m_subsystem.getYaw();

  }

  @Override
  public void execute() {
    // double currentAngle = m_subsystem.getRotation2d().getDegrees();
    // double currentError = Math.abs(currentAngle - targetTheta);

    // //check which way is closer to target
    // boolean turnClockwise = ((currentAngle + 180) % 360) > targetTheta;

    currentTheta = m_subsystem.getRotation2d().getRadians();
    System.out.println("target" + targetTheta);
    System.out.println("current" + currentTheta);
    driveThetad = turningPID.calculate(targetTheta.get(), currentTheta);
    System.out.println("drive speed " + driveThetad);
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