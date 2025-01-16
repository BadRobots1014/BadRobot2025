// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  ShuffleboardTab auto_tab = Shuffleboard.getTab("auto");

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
    new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_auxController =
    new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    boolean isCompetition = false;
    //TODO: set isCompetition to true for comp

    swerveSubsystem = new SwerveSubsystem(m_driverController.getHID());
    swerveSubsystem.setDefaultCommand(new DriveFromControllerCommand(swerveSubsystem, this::getLeftX, this::getLeftY, this::getRightX, this::getSlowMode, this::getFasterMode, this::getPOV));

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    auto_tab.add("pick auto", autoChooser);


    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.R1().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  boolean getFasterMode() {
    return m_driverController.getHID().getL2Button();
  }

  //marcus was here

  boolean getSlowMode() {
    return m_driverController.getHID().getR2Button();
  }

  double getRightX() {return m_driverController.getRightX();}
  double getLeftX() {return -m_driverController.getLeftX();}
  double getLeftY() {return -m_driverController.getLeftY();}
  double getPOV() {return m_driverController.getHID().getPOV();}
  boolean getRightTrigger() {return m_driverController.getHID().getR2Button();}
  double getAuxRightY() {return Math.abs(m_auxController.getRightY()) > OIConstants.kJoystickDeadband ? m_auxController.getRightY() : 0;}
  double getAuxLeftY() {return Math.abs(m_auxController.getLeftY()) > OIConstants.kJoystickDeadband ? m_auxController.getLeftY() : 0;}
  double getAuxPOV() {return m_auxController.getHID().getPOV();}
  double getAuxLTrig(){return m_auxController.getL2Axis();}
  double getAuxRTrig(){return m_auxController.getR2Axis();}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
