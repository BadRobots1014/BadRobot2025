package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveFromControllerCommand extends SwerveDriveCommand {

  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, pov, auxLTrig, auxRTrig;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public DriveFromControllerCommand(
    SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    boolean fieldOriented,
    Supplier<Boolean> fastMode,
    Supplier<Boolean> fasterMode,
    Supplier<Double> povSupplier,
    Supplier<Double> auxLeftTrigger,
    Supplier<Double> auxRightTrigger) {
      
      super(swerveSubsystem, fieldOriented);

      xSpdFunction = xSupplier;
      ySpdFunction = ySupplier;
      turningSpdFunction = turnSupplier;
      fieldOrientedFunction = fieldOriented;
      fastModeFunction = fastMode;
      fasterModeFunction = fasterMode;
      pov = povSupplier;
      auxLTrig = auxLeftTrigger;
      auxRTrig = auxRightTrigger;
      xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
      yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
      turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
  }

  @Override
  public void execute() {
    // Get controller inputs
    double xSpeed = 0, ySpeed = 0, turningSpeed = 0;
    boolean fastMode = false, fasterMode = false;
    if (pov.get() == -1) {
      xSpeed = xSpdFunction.get();
      ySpeed = ySpdFunction.get();
      turningSpeed = turningSpdFunction.get();
      fastMode = fastModeFunction.get();
      fasterMode = fasterModeFunction.get();
    }
    else { //nudge if pov is being inputted
      xSpeed = pov.get() == 90 ? -DriveConstants.kNudgeSpeed : (pov.get() == 270 ? DriveConstants.kNudgeSpeed : 0);
      ySpeed = pov.get() == 0 ? DriveConstants.kNudgeSpeed : (pov.get() == 180 ? -DriveConstants.kNudgeSpeed : 0);
    }

    // Apply speed deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    // Slew rates
    double maxDriveSpeed = fasterMode ? DriveConstants.kFasterTeleMaxMetersPerSec : (fastMode ? DriveConstants.kFastTeleMaxMetersPerSec : DriveConstants.kTeleMaxMetersPerSec);
    double maxTurnSpeed = fasterMode ? DriveConstants.kFasterTeleMaxRadiansPerSec : (fastMode ? DriveConstants.kFastTeleMaxRadiansPerSec : DriveConstants.kTeleMaxRadiansPerSec);
    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * maxTurnSpeed;

    //these lines tell swerve drive command the adjusted inputs to drive
    super.setDriveSpeeds(xSpeed, ySpeed, maxTurnSpeed, pov.get() != -1);
    super.execute();
  }
}
