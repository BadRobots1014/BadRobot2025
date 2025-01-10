package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveControlRoutingSubsystem extends SubsystemBase {
  
  //this subsystem is set as the default command, and keeps track of what is currently driving
  //there will be seperate commands for each driving method that inherit from SwerveDriveCommand
  public DriveControlRoutingSubsystem() {}

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
