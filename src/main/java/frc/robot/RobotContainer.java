// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AuxControllerConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CoralControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HexControllerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlignToCoralCommand;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightPathCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TestModuleCommand;
import frc.robot.commands.TestOdometry;
import frc.robot.commands.TurnToThetaCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.commands.SnapToThetaCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorCommand;
import frc.robot.util.Elastic;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);

  // The Aux Controller Panel
  private Joystick[] auxJoysticks = {
      new Joystick(ControllerConstants.kSecondControllerPortOne),
      new Joystick(ControllerConstants.kSecondControllerPortTwo)
  };

  // Coral Buttons On AUX controller
  JoystickButton level1Left = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kLeftLevel1);
  JoystickButton level1Right = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kRightLevel1);
  JoystickButton level2Left = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kLeftLevel2);
  JoystickButton level2Right = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kRightLevel2);
  JoystickButton level3Left = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kLeftLevel3);
  JoystickButton level3Right = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kRightLevel3);
  JoystickButton level4Left = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kLeftLevel4);
  JoystickButton level4Right = new JoystickButton(auxJoysticks[1], CoralControllerConstants.kRightLevel4);

  // Other Buttons On AUX controller
  JoystickButton AuxLeftBottom = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kLeftBottom);
  JoystickButton AuxRightBottom = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kRightBottom);
  JoystickButton AuxLeftLowerMid = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kLeftLowerMid);
  JoystickButton AuxRightLowerMid = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kRightLowerMid);
  JoystickButton AuxLeftUpperMid = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kLeftUpperMid);
  JoystickButton AuxRightUpperMid = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kRightUpperMid);
  JoystickButton AuxLeftTop = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kLeftTop);
  JoystickButton AuxRightTop = new JoystickButton(auxJoysticks[0], AuxControllerConstants.kRightTop);

  // Hex Buttons On AUX controller
  JoystickButton HexTopLeft = new JoystickButton(auxJoysticks[0], HexControllerConstants.kTopLeft);
  JoystickButton HexTop = new JoystickButton(auxJoysticks[0], HexControllerConstants.kTop);
  JoystickButton HexTopRight = new JoystickButton(auxJoysticks[0], HexControllerConstants.kTopRight);
  JoystickButton HexBottomRight = new JoystickButton(auxJoysticks[1], HexControllerConstants.kBottomRight);
  JoystickButton HexBottom = new JoystickButton(auxJoysticks[1], HexControllerConstants.kBottom);
  JoystickButton HexBottomLeft = new JoystickButton(auxJoysticks[1], HexControllerConstants.kBottomLeft);

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(m_driverController.getHID());
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  private final Command m_leftLevel1Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionLeft),
      new ElevatorCommand(m_elevatorSubsystem, () -> 1));
  private final Command m_rightLevel1Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionRight),
      new ElevatorCommand(m_elevatorSubsystem, () -> 1));
  private final Command m_leftLevel2Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionLeft),
      new ElevatorCommand(m_elevatorSubsystem, () -> 2));
  private final Command m_rightLevel2Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionRight),
      new ElevatorCommand(m_elevatorSubsystem, () -> 2));
  private final Command m_leftLevel3Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionLeft),
      new ElevatorCommand(m_elevatorSubsystem, () -> 3));
  private final Command m_rightLevel3Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionRight),
      new ElevatorCommand(m_elevatorSubsystem, () -> 3));
  private final Command m_leftLevel4Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionLeft),
      new ElevatorCommand(m_elevatorSubsystem, () -> 4));
  private final Command m_rightLevel4Command = Commands.parallel(
      new AlignToCoralCommand(m_swerveSubsystem, CoralControllerConstants.directionRight),
      new ElevatorCommand(m_elevatorSubsystem, () -> 4));

  /*
   * private final AlignToCoralCommand m_leftAlignToCoralCommand = new
   * AlignToCoralCommand(m_swerveSubsystem, -1);
   * private final AlignToCoralCommand m_rightAlignToCoralCommand = new
   * AlignToCoralCommand(m_swerveSubsystem, 1);
   * 
   * private final ElevatorCommand m_level1ElevatorCommand = new
   * ElevatorCommand(m_elevatorSubsystem, () -> 1);
   * private final ElevatorCommand m_level2ElevatorCommand = new
   * ElevatorCommand(m_elevatorSubsystem, () -> 2);
   * private final ElevatorCommand m_level3ElevatorCommand = new
   * ElevatorCommand(m_elevatorSubsystem, () -> 3);
   * private final ElevatorCommand m_level4ElevatorCommand = new
   * ElevatorCommand(m_elevatorSubsystem, () -> 4);
   */

  // private final SendableChooser<Command> autoChooser;

  // private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

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
        this::getPOV));
        //this::getAuxLeftTrigger,
        //this::getAuxRightTrigger));
    // m_swerveSubsystem.setDefaultCommand(new TestModuleCommand(m_swerveSubsystem,
    // new SwerveModuleState[] {
    // new SwerveModuleState(1, Rotation2d.fromDegrees(0)), // FL
    // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // FR
    // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // BL
    // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // BR
    // }));

    m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand(m_elevatorSubsystem, this::getRightY, true));

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
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.options().whileTrue(new ZeroHeadingCommand(m_swerveSubsystem));
    m_driverController.R2().whileTrue(new AlignToTargetCommand(m_limelightSubsystem, m_swerveSubsystem, m_driverController.getHID()));
    m_driverController.L2().whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> this.getRightAngle(), () -> getLeftX(), () -> getLeftY(), true, () -> this.angleRelevant()));
    m_driverController.L1().whileTrue(new SnapToThetaCommand(m_swerveSubsystem, () -> this.getRightAngle(), () -> getLeftX(), () -> getLeftY(), true, () -> this.angleRelevant(), 60));

    //m_driverController.L1().whileTrue(new TestOdometry(m_swerveSubsystem, 1, 0));
    m_driverController.triangle().onTrue(new LimelightPathCommand(m_swerveSubsystem, () -> 2d, () -> 0d, () -> Rotation2d.fromDegrees(90)));
    m_driverController.square().onTrue(new LimelightPathCommand(m_swerveSubsystem, m_limelightSubsystem));
    // m_driverController.options().whileTrue(new
    // ZeroHeadingCommand(m_swerveSubsystem));
    // m_driverController.R2().whileTrue(new
    // AlignToTargetCommand(m_limelightSubsystem, m_swerveSubsystem,
    // m_driverController.getHID()));
    // m_driverController.L2().whileTrue(new TurnToThetaCommand(m_swerveSubsystem,
    // () -> this.getRightAngle(), () -> getLeftX(), () -> getLeftY(), true, () ->
    // this.angleRelevant()));
    // m_driverController.cross().whileTrue(new AlgaeCommand(m_algaeSubsystem,
    // true)); // Uncomment when set algae speeds are determined
    // m_driverController.square().whileTrue(new AlgaeCommand(m_algaeSubsystem,
    // false));

    level1Left.onTrue(m_leftLevel1Command);
    level1Right.onTrue(m_rightLevel1Command);
    level2Left.onTrue(m_leftLevel2Command);
    level2Right.onTrue(m_rightLevel2Command);
    level3Left.onTrue(m_leftLevel3Command);
    level3Right.onTrue(m_rightLevel3Command);
    level4Left.onTrue(m_leftLevel4Command);
    level4Right.onTrue(m_rightLevel4Command);

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
    }
    else fasterMode = false;
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
    return autoChooser.getSelected();
  }
}