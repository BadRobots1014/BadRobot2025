// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AuxControllerConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralMode;
import frc.robot.Constants.CoralControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HexControllerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightPathCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TestModuleCommand;
import frc.robot.commands.TestOdometry;
import frc.robot.commands.TurnToThetaCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.commands.SnapToThetaCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorCommand;
import frc.robot.util.Elastic;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BlinkinCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CoralCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;

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

  private BlinkinSubsystem blinkinSubsystem;
  private BlinkinCommand redBlinkinCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private Joystick[] auxJoysticks = {
    new Joystick(ControllerConstants.kSecondControllerPortOne),
    new Joystick(ControllerConstants.kSecondControllerPortTwo)
};

CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);

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
private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
private final WinchSubsystem m_winchSubsystem = new WinchSubsystem();



private final Command m_level1Command = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlOnePos);
private final Command m_level2Command = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlTwoPos);
private final Command m_level3Command = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlThreePos);
private final Command m_level4Command = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlFourPos);

public final Command m_level1CommandTimeOut = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlOnePos).withTimeout(3);
public final Command m_level2CommandTimeOut = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlTwoPos).withTimeout(3);
public final Command m_level3CommandTimeOut = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlThreePos).withTimeout(3);
public final Command m_level4CommandTimeOut = new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kLvlFourPos).withTimeout(3);

public final Command m_coralDumpCommand = new CoralCommand(m_coralSubsystem, () -> CoralConstants.kCoralDownSpeed, () -> true);
public final Command m_coralUndumpCommand = new CoralCommand(m_coralSubsystem, () -> CoralConstants.kCoralUpSpeed, () -> true);
public final Command m_coralDumpCommandTimeOut = m_coralDumpCommand.withTimeout(1);
public final Command m_coralUndumpCommandTimeOut = m_coralUndumpCommand.withTimeout(1);

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

  private final SendableChooser<Command> autoChooser;

  boolean fastMode = true, fasterMode = false, toggled = false;
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      this.blinkinSubsystem = new BlinkinSubsystem();
  
      this.redBlinkinCommand = new BlinkinCommand(blinkinSubsystem, .61);
  
      // Configure the trigger bindings
      m_swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_swerveSubsystem,
          () -> getLeftY(),
          () -> getLeftX(),
          () -> getRightX(),
          DriveConstants.kFieldOriented,
          this::getFastMode,
          this::getFasterMode,
          this::getPOV,
          () -> 0d,
          () -> 0d));
      // m_swerveSubsystem.setDefaultCommand(new TestModuleCommand(m_swerveSubsystem,
      // new SwerveModuleState[] {
      // new SwerveModuleState(1, Rotation2d.fromDegrees(0)), // FL
      // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // FR
      // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // BL
      // new SwerveModuleState(0, Rotation2d.fromDegrees(0)), // BR
      // }));

      // Add named commands to pathplanner
      NamedCommands.registerCommand("Move to L1", m_level1CommandTimeOut);
      NamedCommands.registerCommand("Move to L2", m_level2CommandTimeOut);
      NamedCommands.registerCommand("Move to L3", m_level3CommandTimeOut);
      NamedCommands.registerCommand("Move to L4", m_level4CommandTimeOut);
      NamedCommands.registerCommand("Dump Coral", m_coralDumpCommandTimeOut);
      NamedCommands.registerCommand("Undump Coral", m_coralUndumpCommandTimeOut);
  
      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
  
      // Start Elastic Server
      WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  
      // Another option that allows you to specify the default auto by its name
      // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  
      SmartDashboard.putData("Auto Chooser", autoChooser);
      SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
      SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  
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
      m_driverController.options().whileTrue(new ZeroHeadingCommand(m_swerveSubsystem));
      level1Left.whileTrue(m_level1Command);
      level2Left.whileTrue(m_level2Command);
      level3Left.whileTrue(m_level3Command);
      level4Left.whileTrue(m_level4Command);
      // AuxLeftBottom.whileTrue(new WinchCommand(m_winchSubsystem, WinchConstants.kWinchDownPower));
      // AuxRightBottom.whileTrue(new WinchCommand(m_winchSubsystem, WinchConstants.kWinchUpPower));

      level4Right.whileTrue(new LimelightPathCommand(m_swerveSubsystem, m_limelightSubsystem));
  
      m_driverController.R1().whileTrue(new AlgaeCommand(m_algaeSubsystem, false));
      m_driverController.L1().whileTrue(new AlgaeCommand(m_algaeSubsystem, true));
      AuxLeftUpperMid.whileTrue(new AlgaeCommand(m_algaeSubsystem, true));
      AuxRightUpperMid.whileTrue(new AlgaeCommand(m_algaeSubsystem, false));
      
      m_driverController.L2().whileTrue(new CoralCommand(m_coralSubsystem, () -> CoralConstants.kCoralDownSpeed, () -> true));
      m_driverController.R2().whileTrue(new CoralCommand(m_coralSubsystem, () -> CoralConstants.kCoralUpSpeed, () -> true));
      AuxLeftTop.whileTrue(new CoralCommand(m_coralSubsystem, () -> CoralConstants.kCoralUpSpeed, () -> true));
      AuxRightTop.whileTrue(new CoralCommand(m_coralSubsystem, () -> CoralConstants.kCoralDownSpeed, () -> true));

      // TODO add override up mode
  
      AuxRightLowerMid.whileTrue(new ClimbCommand(m_climberSubsystem, () -> 1d));
      AuxLeftLowerMid.whileTrue(new ClimbCommand(m_climberSubsystem, () -> -1d));
  
      m_driverController.triangle().whileTrue(new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kElevatorUpPower, true));
      m_driverController.cross().whileTrue(new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kElevatorDownPower, true));
      AuxLeftBottom.whileTrue(new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kElevatorUpPower, true));
      AuxRightBottom.whileTrue(new ElevatorCommand(m_elevatorSubsystem, () -> ElevatorConstants.kElevatorDownPower, true));
  
      //Reef angle presets
      HexTopLeft.whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> Math.toRadians(240), () -> getLeftX(), () -> getLeftY(), true, () -> true));
      HexTop.whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> Math.toRadians(180), () -> getLeftX(), () -> getLeftY(), true, () -> true));
      HexTopRight.whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> Math.toRadians(120), () -> getLeftX(), () -> getLeftY(), true, () -> true));
      HexBottomLeft.whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> Math.toRadians(300), () -> getLeftX(), () -> getLeftY(), true, () -> true));
      HexBottom.whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> Math.toRadians(0), () -> getLeftX(), () -> getLeftY(), true, () -> true));
      HexBottomRight.whileTrue(new TurnToThetaCommand(m_swerveSubsystem, () -> Math.toRadians(60), () -> getLeftX(), () -> getLeftY(), true, () -> true));
    }
  
    boolean getFastMode() {
      if (m_driverController.share().getAsBoolean() && !toggled) {
        toggled = true;
      fastMode = !fastMode;
    }
    else if (!m_driverController.share().getAsBoolean() && toggled) {
      toggled = false;
    }
    return fastMode;
  }

  boolean getFasterMode() {
    if (m_driverController.getL2Axis() > OIConstants.kTriggerDeadband) fasterMode = true;
    else fasterMode = false;
    return true;
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