package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class TestOdometry extends SwerveDriveCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  SwerveSubsystem m_subsystem;

  private static double driveXd = 0;
  private static double driveYd = 0;

  private double targetXDistance = 0;
  private double targetYDistance = 0;

  private Pose2d startingPosition;
  private Pose2d currentPosition;
  private Pose2d distanceTravelled;

  private PIDController drivePID = new PIDController(3, 0, 0);
  
  ShuffleboardTab m_tab;

  private static Supplier<Double> driveX = new Supplier<Double>() {
    @Override
    public Double get() {
      return driveXd;
    }
  };

  private static Supplier<Double> driveY = new Supplier<Double>() {
    @Override
    public Double get() {
      return driveYd;
    }
  };

  public TestOdometry(SwerveSubsystem swerveSubsystem, double targetXDistance, double targetYDistance) {
    super(
        swerveSubsystem,
        driveX,
        driveY,
        swerveSubsystem.thetaHelper.driveTheta,
        true,
        () -> false,
        () -> false,
        () -> -1d,
        () -> 0d,
        () -> 0d);

    m_subsystem = swerveSubsystem;
    drivePID = new PIDController(1.4, 0, 0);

    this.targetXDistance = targetXDistance;
    this.targetYDistance = targetYDistance;
    startingPosition = m_subsystem.getOdomentryPose();

    m_tab = Shuffleboard.getTab("Limelight");

    m_tab.addString("Starting Position", () -> startingPosition.toString());
    m_tab.addDouble("Current X", () -> currentPosition.getX());
    m_tab.addDouble("Current Y", () -> currentPosition.getY());
    m_tab.addDouble("Distance X Travelled", () -> distanceTravelled.getX());
    m_tab.addDouble("Distance Y Travelled", () -> distanceTravelled.getY());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    currentPosition = m_subsystem.getOdomentryPose();
    distanceTravelled = currentPosition.relativeTo(startingPosition);

    double x = distanceTravelled.getX();
    driveXd = drivePID.calculate(targetXDistance, x);

    double y = distanceTravelled.getY();
    driveYd = drivePID.calculate(targetYDistance, y);
    
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}