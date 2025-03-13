// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TestModuleCommand;
import frc.robot.commands.TestOdometry;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.commands.TurnToThetaCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);
  private final PS4Controller m_auxController = new PS4Controller(OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(m_driverController.getHID());
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  // private final SendableChooser<Command> autoChooser;

  boolean fastMode = false, fasterMode = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_swerveSubsystem,
        () -> getLeftY(),
        () -> getLeftX(),
        () -> getRightX(),
        DriveConstants.kFieldOriented,
        this::getFastMode,
        this::getFasterMode,
        this::getPOV,
        this::getAuxLeftTrigger,
        this::getAuxRightTrigger));
    // m_swerveSubsystem.setDefaultCommand(new TestModuleCommand(m_swerveSubsystem,
    // new SwerveModuleState[] {
    // new SwerveModuleState(1, Rotation2d.fromDegrees(0)), // FL
    // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // FR
    // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // BL
    // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // BR
    // }));

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Start Elastic Server
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    // SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.options().whileTrue(new
    // ZeroHeadingCommand(m_swerveSubsystem));
    // m_driverController.R2().whileTrue(new
    // AlignToTargetCommand(m_limelightSubsystem, m_swerveSubsystem,
    // m_driverController.getHID()));
    // m_driverController.L2().whileTrue(new TurnToThetaCommand(m_swerveSubsystem,
    // () -> this.getRightAngle(), () -> getLeftX(), () -> getLeftY(), true, () ->
    // this.angleRelevant()));
    m_driverController.cross().whileTrue(new AlgaeCommand(m_algaeSubsystem,
        true)); // Uncomment when set algae speeds are determined
    m_driverController.square().whileTrue(new AlgaeCommand(m_algaeSubsystem,
        false));
  }

  boolean getFastMode() {
    if (m_driverController.getHID().getL1Button()) {
      fastMode = !fastMode;
    }
    return fastMode;
  }

  boolean getFasterMode() {
    if (m_driverController.getL2Axis() > OIConstants.kTriggerDeadband) {
      fasterMode = true;
    } else
      fasterMode = false;
    return fasterMode;
  }

  double getRightX() {
    return Math.abs(m_driverController.getRightX()) >= 0.1 ? m_driverController.getRightX() : 0;
  }

  double getRightY() {
    return Math.abs(m_driverController.getRightY()) >= 0.1 ? m_driverController.getRightY() : 0;
  }

  double getLeftX() {
    return m_driverController.getLeftX();
  }

  double getLeftY() {
    return -m_driverController.getLeftY();
  }

  double getPOV() {
    return m_driverController.getHID().getPOV() == -1 ? m_driverController.getHID().getPOV()
        : (m_driverController.getHID().getPOV() + 270) % 360;
  }

  double getAuxRightY() {
    return Math.abs(m_auxController.getRightY()) > OIConstants.kDriveDeadband ? m_auxController.getRightY() : 0;
  }

  double getAuxLeftY() {
    return Math.abs(m_auxController.getLeftY()) > OIConstants.kDriveDeadband ? m_auxController.getLeftY() : 0;
  }

  double getAuxPOV() {
    return m_auxController.getPOV();
  }

  double getAuxLeftTrigger() {
    return m_auxController.getL2Axis();
  }

  double getAuxRightTrigger() {
    return m_auxController.getR2Axis();
  }

  double getRightAngle() {
    return Math.atan2(this.getRightX(), -this.getRightY());
  }

  boolean angleRelevant() {
    return Math.pow(getRightX(), 2) + Math.pow(getRightY(), 2) >= 0.2;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}