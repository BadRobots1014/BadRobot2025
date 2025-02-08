package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToThetaCommand extends SwerveDriveCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  static double driveThetad;
  SwerveSubsystem m_subsystem;

  double currentTheta;
  Supplier<Double> targetTheta;

  private static Supplier<Boolean> angleRelevant;

  private static Rotation2d lastTheta = new Rotation2d();

  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta, Supplier<Double> moveX,
      Supplier<Double> moveY, boolean fieldOriented, Supplier<Boolean> angleRelevant) {
    super(
        swerveSubsystem,
        moveX,
        moveY,
        swerveSubsystem.thetaHelper.driveTheta,
        fieldOriented,
        () -> false,
        () -> false,
        () -> -1d,
        () -> 0d,
        () -> 0d);
    this.targetTheta = targetTheta;
    m_subsystem = swerveSubsystem;

    this.angleRelevant = angleRelevant;
  }

  @Override
  public void initialize() {
    lastTheta = m_subsystem.getRotation2d();
  }

  @Override
  public void execute() {
    // double currentAngle = m_subsystem.getRotation2d().getDegrees();
    // double currentError = Math.abs(currentAngle - targetTheta);

    // //check which way is closer to target
    // boolean turnClockwise = ((currentAngle + 180) % 360) > targetTheta;

    // Begin rotating to target theta
    if (angleRelevant.get()){
      lastTheta = new Rotation2d(targetTheta.get());
    }
    Rotation2d currentTheta = m_subsystem.getRotation2d();
    m_subsystem.thetaHelper.calculate(currentTheta, lastTheta);

    //System.out.println("target" + targetTheta);
    //System.out.println("current" + currentTheta);
    //System.out.println("drive speed " + m_subsystem.thetaHelper.driveTheta.get());

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}