package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.config.ModuleConfig;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToThetaCommand extends DriveFromControllerCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_subsystem;
  private int targetTheta;

  public TurnToThetaCommand(
    SwerveSubsystem swerveSubsystem,
    int targetTheta,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Boolean> fastMode,
    Supplier<Boolean> fasterMode
  ) {
    super(
      swerveSubsystem, 
      xSupplier, 
      ySupplier, 
      () -> {return 0d;}, //value from turn joystick isnt needed
      fastMode, 
      fasterMode, 
      () -> {return -1d;} //nudge also isnt needed
    );

    m_subsystem = swerveSubsystem;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // set starting values
    super.turningSpeed = DriveConstants.kTurnThetaFastSpeed;

    double currentAngle = m_subsystem.getRotation2d().getDegrees();
    double currentError = Math.abs(currentAngle - targetTheta);

    //check which way is closer to target
    boolean turnClockwise = ((currentAngle + 180) % 360) > targetTheta;

    //set speed depending on how far we are from target
    if (currentError < OIConstants.kJoystickDeadband)
      super.turningSpeed = 0;
    else if (currentError < DriveConstants.kTurnThetaErrorForSlowSpeed)
      super.turningSpeed = DriveConstants.kTurnThetaSlowSpeed;

    //apply counter clockwise turn if needed
    if (!turnClockwise)
      super.turningSpeed = -super.turningSpeed;

    //set speeds
    super.calculateTurn();
    super.calculateDrive();

    super.setDriveSpeeds();
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
