package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

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

  private static Supplier<Boolean> angleRelevant;

  private static double lastTheta = 0;

  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta, Supplier<Double> moveX,
      Supplier<Double> moveY, boolean fieldOriented) {
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
        () -> 0d);
    this.targetTheta = targetTheta;
    m_subsystem = swerveSubsystem;

    turningPID = new PIDController(1, 0, 0);
    turningPID.enableContinuousInput(0, 2 * Math.PI);

    angleRelevant = () -> true;
  }

  public TurnToThetaCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> targetTheta, Supplier<Double> moveX,
      Supplier<Double> moveY, boolean fieldOriented, Supplier<Boolean> angleRelevant) {
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
        () -> 0d);
    this.targetTheta = targetTheta;
    m_subsystem = swerveSubsystem;

    turningPID = new PIDController(1, 0, 0);
    turningPID.enableContinuousInput(0, 2 * Math.PI);

    this.angleRelevant = angleRelevant;
  }

  @Override
  public void initialize() {
    lastTheta = m_subsystem.getRotation2d().getRadians();
  }

  @Override
  public void execute() {
    if (angleRelevant.get()) {
      currentTheta = m_subsystem.getRotation2d().getRadians();
      lastTheta = targetTheta.get();
      System.out.println("target" + targetTheta.get());
      System.out.println("current" + currentTheta);
      driveThetad = turningPID.calculate(targetTheta.get(), currentTheta);
      super.execute();
    } else {
      currentTheta = m_subsystem.getRotation2d().getRadians();
      System.out.println("target" + lastTheta);
      System.out.println("current" + currentTheta);
      driveThetad = turningPID.calculate(lastTheta, currentTheta);
      super.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
