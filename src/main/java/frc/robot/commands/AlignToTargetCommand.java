package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlignToTargetCommand extends SwerveDriveCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  // Required so camera setup is completed
  private final LimelightSubsystem m_limelight_subsystem;

  private static double driveXd = 0;
  private static double driveYd = 0;
  private Rotation2d targetTheta = new Rotation2d();

  private Pose3d lastPosLimelight; // Last read position relative to tag
  private double lastTag; // The last tag ID read
  private Pose2d lastDisplacement; // last position the robot was at when it saw a tag

  private PS4Controller m_controller;

  private PIDController drivePID;

  double remainingXDistance;
  double remainingYDistance;

  ShuffleboardTab m_tab;

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

  // private static Supplier<Double> targetTheta = new Supplier<Double>() {
  // @Override
  // public Double get() {
  // return targetThetad;
  // }
  // };

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

    drivePID = new PIDController(1.4, 0, 0);

    lastDisplacement = new Pose2d();
    lastPosLimelight = new Pose3d();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight_subsystem);

    m_tab = Shuffleboard.getTab("Limelight");

    m_tab.addDouble("Current Displacement X", () -> swerveSubsystem.getGruPose().getX());
    m_tab.addDouble("Current Displacement Y", () -> swerveSubsystem.getGruPose().getY());
    m_tab.addDouble("Current Displacement Rotation", () -> swerveSubsystem.getGruPose().getRotation().getDegrees());
    m_tab.addString("Last Displacement", () -> lastDisplacement.toString());
    m_tab.addString("Limelight Previous", () -> lastPosLimelight.toString());

    m_tab.addDouble("Remaining Y distance", () -> remainingYDistance);
    m_tab.addDouble("Remaining X distance", () -> remainingXDistance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // // Since the supplier will always point to the target value, it can be set
    // // prematurely
    // swerveSubsystem.thetaHelper.setTargetTheta(targetTheta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastTag = LimelightHelpers.getFiducialID("");
    // If it actively sees an april tag
    if (lastTag != -1) {
      // set last position
      lastPosLimelight = LimelightHelpers.getBotPose3d_TargetSpace("");

      // set last displacement
      lastDisplacement = swerveSubsystem.m_odometry.getPoseMeters();

      // get current rotation for turning
      Rotation2d currentTheta = swerveSubsystem.getRotation2d();

      // Use known april tag orientations to know current angle in radians
      double targetRadians = DriveConstants.aprilTagAngles.get(lastTag) * Math.PI / 180;
      //targetTheta = aprilTagAngles.get(lastTag)*Math.PI/180;

      // Run PID to compute speed
      swerveSubsystem.thetaHelper.calculate(currentTheta, targetTheta);

      // Take the smaller speed depending on direction
      double x = -lastPosLimelight.getX();
      driveXd = drivePID.calculate(0d, x);

      // Z in 3d space corrosponds to the Y for the motor
      double y = lastPosLimelight.getZ();
      // driveYd = 0;
      driveYd = drivePID.calculate(DriveConstants.kAutoTargetDistance, y);
    } else {
      // get current rotation for turning
      Rotation2d currentTheta = swerveSubsystem.getRotation2d();

      // Run PID to compute speed
      swerveSubsystem.thetaHelper.calculate(currentTheta, targetTheta);

      Pose2d currentDisplacement = swerveSubsystem.m_currentDisplacement;

      // total distance - distance travelled
      remainingXDistance = lastPosLimelight.getX() - (lastDisplacement.getX() - currentDisplacement.getX());
      remainingYDistance = lastPosLimelight.getZ() - (lastDisplacement.getY() - currentDisplacement.getY());

      driveXd = drivePID.calculate(0d, -remainingXDistance);
      // driveYd = 0;

      driveYd = drivePID.calculate(DriveConstants.kAutoTargetDistance, remainingYDistance);
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
