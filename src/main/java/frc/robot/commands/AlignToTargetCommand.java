package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;

public class AlignToTargetCommand extends SwerveDriveCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  // Required so camera setup is completed
  private final LimelightSubsystem m_limelight_subsystem;

  private static double driveXd = 0;
  private static double driveYd = 0;
  private static double targetThetad = 0;

  private static Pose3d lastPos;
  private static double lastTag;

  private PS4Controller m_controller;

  ShuffleboardTab m_tab = Shuffleboard.getTab("Limelight");

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

  private static Supplier<Double> targetTheta = new Supplier<Double>() {
    @Override
    public Double get() {
      return targetThetad;
    }
  };

  public AlignToTargetCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem,
      PS4Controller controller) {
    // SwerveSubsystem subsystem,
    // Supplier<Double> xSupplier,
    // Supplier<Double> ySupplier,
    // Supplier<Double> turnSupplier,
    // boolean fieldOriented,
    // Supplier<Boolean> fastMode,
    // Supplier<Boolean> fasterMode,
    // Supplier<Double> povSupplier,
    // Supplier<Double> auxLeftTrigger,
    // Supplier<Double> auxRightTrigger
    super(
        swerveSubsystem,
        driveX,
        driveY,
        swerveSubsystem.thetaHelper.driveTheta,
        false,
        () -> false,
        () -> false,
        () -> -1d,
        () -> 0d,
        () -> 0d);

    m_controller = controller;

    m_limelight_subsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Since the supplier will always point to the target value, it can be set
    // prematurely
    swerveSubsystem.thetaHelper.setTargetTheta(targetTheta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastTag = LimelightHelpers.getFiducialID("");

    // If it actively sees an april tag
    if (lastTag != -1) {
      // set last position
      lastPos = LimelightHelpers.getBotPose3d_TargetSpace("");

      // get current rotation for turning
      Rotation2d currentTheta = swerveSubsystem.getRotation2d();

      // Returns the yaw even though it says pitch
      double yaw = lastPos.getRotation().getY();

      // Computes what angle the robot has to be to face the april tag
      targetThetad = currentTheta.getRadians() - yaw;

      // Run PID to compute speed
      swerveSubsystem.thetaHelper.calculate(currentTheta);

      // Take the smaller speed depending on direction
      double x = lastPos.getX();
      driveXd = -MathUtil.clamp(x, -1, 1);

      // Z in 3d space corrosponds to the Y for the motor
      driveYd = Math.min(lastPos.getZ(), 1);

    } else {
    }

    /*
     * GET READY TO RUUUUMMMMBBBLLLLEEEEE
     * if (yaw > 0) {
     * m_controller.setRumble(RumbleType.kLeftRumble, 0.5);
     * m_controller.setRumble(RumbleType.kRightRumble, 0);
     * } else if (yaw < 0) {
     * m_controller.setRumble(RumbleType.kRightRumble, 0.5);
     * m_controller.setRumble(RumbleType.kLeftRumble, 0);
     * }
     */
    super.execute();

  }

  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleType.kRightRumble, 0);
    m_controller.setRumble(RumbleType.kLeftRumble, 0);
    super.end(interrupted);
  }
}
