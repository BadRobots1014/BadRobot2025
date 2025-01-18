package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveFromControllerCommand extends SwerveDriveCommand {

  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, pov;
  public final Supplier<Boolean> slowModeFunction, fasterModeFunction;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public double xSpeed = 0, ySpeed = 0, turningSpeed = 0;
  public boolean slowMode = false, fasterMode = false;

  public DriveFromControllerCommand(
    SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    Supplier<Boolean> slowMode,
    Supplier<Boolean> fasterMode,
    Supplier<Double> povSupplier) {
      
      super(swerveSubsystem);

      xSpdFunction = xSupplier;
      ySpdFunction = ySupplier;
      turningSpdFunction = turnSupplier;
      slowModeFunction = slowMode;
      fasterModeFunction = fasterMode;
      pov = povSupplier;

      xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
      yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
      turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
  }

  @Override
  public void execute() {
    // Get controller inputs
    if (pov.get() == -1) {
      xSpeed = xSpdFunction.get();
      ySpeed = ySpdFunction.get();
      turningSpeed = turningSpdFunction.get();
      slowMode = slowModeFunction.get();
      fasterMode = fasterModeFunction.get();

      //we dont need to calculate turn if the robot is being nudged
      calculateTurn();
    }
    else { //nudge if pov is being inputted
      xSpeed = pov.get() == 90 ? -DriveConstants.kNudgeSpeed : (pov.get() == 270 ? DriveConstants.kNudgeSpeed : 0);
      ySpeed = pov.get() == 0 ? DriveConstants.kNudgeSpeed : (pov.get() == 180 ? -DriveConstants.kNudgeSpeed : 0);
    }

    //we always need to calculate drive and set the values
    calculateDrive();
    setDriveSpeeds();
  }

  public void calculateDrive() {
    // Apply speed deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kJoystickDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kJoystickDeadband ? ySpeed : 0;

    // Slew rates
    double maxDriveSpeed = fasterMode ? DriveConstants.kFasterTeleMaxMetersPerSec : (slowMode ? DriveConstants.kSlowTeleMaxMetersPerSec : DriveConstants.kTeleMaxMetersPerSec);
    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
  }

  public void calculateTurn() {
    // turning deadband
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kJoystickDeadband ? turningSpeed : 0;

    // Slew rates
    double maxTurnSpeed = fasterMode ? DriveConstants.kFasterTeleMaxRadiansPerSec : (slowMode ? DriveConstants.kSlowTeleMaxRadiansPerSec : DriveConstants.kTeleMaxRadiansPerSec);
    turningSpeed = turningLimiter.calculate(turningSpeed) * maxTurnSpeed;
  }

  public void setDriveSpeeds()
  {
    //these lines tell swerve drive command the adjusted inputs to drive
    super.setDriveSpeeds(xSpeed, ySpeed, turningSpeed, pov.get() != -1);
    super.execute();
  }
}
