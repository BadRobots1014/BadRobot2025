package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveControlRoutingSubsystem extends SubsystemBase {
  
  public CurrentDriveInputs currentDriver = CurrentDriveInputs.CONTROLLER;

  //this subsystem is set as the default command, and keeps track of what is currently driving
  //there will be seperate commands for each driving method that inherit from SwerveDriveCommand
  public DriveControlRoutingSubsystem() {}

  public boolean isControllerDriving() {
    return currentDriver == CurrentDriveInputs.CONTROLLER;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public enum CurrentDriveInputs {
    CONTROLLER, //all inputs are raw from controller
    LIMELIGHT, //limelight is lining up with april tag
    TURN_THETA, //turning is hijacked and set to an angle
    PATHPLANNER, //auto or moving to waypoint (most likely won't use waypoint functionality)
  }
}
