package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

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
  
  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta, Supplier<Double> moveX, Supplier<Double> moveY, boolean fieldOriented)
  {
    super(
      swerveSubsystem,
      moveX,
      moveY,
      driveTheta,
      fieldOriented,
      () -> false,
      () -> false,
      () -> -1d,
      () -> 0d,
      () -> 0d
    );
    this.targetTheta = targetTheta;
    m_subsystem = swerveSubsystem;

    turningPID = new PIDController(1,0, 0);
    turningPID.enableContinuousInput(0, 2*Math.PI);
  }

  @Override
  public void initialize() {
    currentTheta = m_subsystem.getYaw();

  }

  @Override
  public void execute() {
    currentTheta = m_subsystem.getRotation2d().getRadians();
    System.out.println("target" + targetTheta.get());
    System.out.println("current" + currentTheta);
    driveThetad = turningPID.calculate(targetTheta.get(), currentTheta);
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
