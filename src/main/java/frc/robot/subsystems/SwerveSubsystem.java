package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.SwerveModule;
import frc.robot.util.TurnThetaHelper;

public class SwerveSubsystem extends SubsystemBase {

  PS4Controller Controller;

  public ShuffleboardTab m_tab = Shuffleboard.getTab("swerve");

  public GenericEntry p = m_tab.add("p", ModuleConstants.kTurningP).getEntry();
  public GenericEntry i = m_tab.add("i", ModuleConstants.kTurningI).getEntry();
  public GenericEntry d = m_tab.add("d", ModuleConstants.kTurningD).getEntry();

  public double offsetX = 0;
  public double offsetY = 0;

  private Field2d m_field = new Field2d();

  // The current position of robot relative to starting position (odometry)
  public Pose2d m_currentDisplacement;

  public SwerveSubsystem(PS4Controller controller) {

    // Delay to allow navx to boot up
    new Thread(() -> {
      try {
        Thread.sleep(DriveConstants.kBootupDelay);
        resetPose();
      } catch (Exception e) {
      }
    }).start();

    SmartDashboard.putNumber("Swerve/Heading", getHeading());
    SmartDashboard.putNumber("Swerve/Yaw", getYaw());
    SmartDashboard.putNumber("Swerve/Roll", getRoll());
    SmartDashboard.putNumber("Swerve/Pitch", getPitch());
    SmartDashboard.putNumber("Swerve/X", getX());
    SmartDashboard.putNumber("Swerve/Y", getY());
    SmartDashboard.putNumber("Swerve/X Offset", offsetX);
    SmartDashboard.putNumber("Swerve/Y Offset", offsetY);
    SmartDashboard.putBoolean("Swerve/NavX isConnected", gru.isConnected());
    SmartDashboard.putBoolean("Swerve/NavX isCalibrating", gru.isCalibrating());

    SmartDashboard.putData(m_field);

    // Create SmartDashboard data for the swerve drive
    SmartDashboard.putData("Swerve/Swerve Modules", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        // Add front left module properties
        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getInvertedAbsoluteRad(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

        // Add front right module properties
        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getInvertedAbsoluteRad(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

        // Add back left module properties
        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getInvertedAbsoluteRad(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

        // Add back right module properties
        builder.addDoubleProperty("Back Right Angle", () -> backRight.getInvertedAbsoluteRad(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

        // Add robot overall angle property
        builder.addDoubleProperty("Robot Angle", () -> 1.6, null);
      }
    });

    Controller = controller;

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::updateOdometry, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveAutoRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(DriveConstants.kAutoTranslationalP, DriveConstants.kAutoTranslationalI, DriveConstants.kAutoTranslationalD), // Translation PID constants
            new PIDConstants(DriveConstants.kAutoRotationalP, DriveConstants.kAutoRotationalI, DriveConstants.kAutoRotationalD) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE"


          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public Command PathToLimelight(Supplier<Double> endX, Supplier<Double> endY, Supplier<Rotation2d> endRotation)
  {

    // Reset everything to zero to start
    resetPose();
    resetOdometry(new Pose2d(3, 3, new Rotation2d()));

    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      getOdomentryPose(),
      getOdomentryPose().transformBy(new Transform2d(-endX.get(), endY.get(), /*endRotation.get())*/ new Rotation2d()))
    );
    
    System.out.println("Pre-flip" + waypoints);

    // Invert and rotate the controls on the waypoints
    //
    // new Translation2d(
    //   waypoints.get(0).nextControl()
    //     .plus(waypoints.get(0).anchor()
    //       .minus(waypoints.get(0).nextControl())
    //         .times(2)).getY(),
    //   waypoints.get(0).nextControl()
    //     .plus(waypoints.get(0).anchor()
    //       .minus(waypoints.get(0).nextControl())
    //         .times(2)).getX()
    //.plus(waypoints.get(1).anchor())
    // new Translation2d(
    //       waypoints.get(1).nextControl()
    //       .plus(waypoints.get(1).anchor()
    //         .minus(waypoints.get(1).nextControl())
    //           .times(2)).getY(),
    //       waypoints.get(1).nextControl()
    //         .plus(waypoints.get(1).anchor()
    //           .minus(waypoints.get(1).nextControl())
    //             .times(2)).getX()
    //     )
    //  .minus(waypoints.get(1).prevControl()).times(2))

    waypoints.set(0,
      new Waypoint(
        waypoints.get(0).prevControl(),
        waypoints.get(0).anchor(),
        new Translation2d(
          waypoints.get(0).nextControl()
            .plus(waypoints.get(0).anchor()
              .minus(waypoints.get(0).nextControl())
                .times(2)).getY(),
          waypoints.get(0).nextControl()
            .plus(waypoints.get(0).anchor()
              .minus(waypoints.get(0).nextControl())
                .times(2)).getX()
        )
        // waypoints.get(0).nextControl().rotateAround(waypoints.get(0).anchor(), Rotation2d.fromDegrees(90))
      )
    );
    waypoints.set(1,
      new Waypoint(
        // new Translation2d(
        //   waypoints.get(1).prevControl()
        //   .plus(waypoints.get(1).anchor()
        //     .minus(waypoints.get(1).prevControl())
        //       .times(2)).getX(),
        //   waypoints.get(1).prevControl()
        //     .plus(waypoints.get(1).anchor()
        //       .minus(waypoints.get(1).prevControl())
        //         .times(2)).getY()
        // ),
        waypoints.get(1).prevControl().rotateAround(waypoints.get(1).anchor(), Rotation2d.fromDegrees(-90)),
        waypoints.get(1).anchor(),
        waypoints.get(1).nextControl()
      )
    );

    System.out.println("Post-flip" + waypoints);

    PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0.0, endRotation.get()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public Command PathToLimelight(Supplier<Double> endX, Supplier<Double> endY) {
    return PathToLimelight(endX, endY, () -> getRotation2d());
  }

  // Modules
  public SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      DriveConstants.kFrontLeftEncoderCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftAbsoluteEncoderReversed,
      p, i, d);

  public SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      DriveConstants.kFrontRightEncoderCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightAbsoluteEncoderReversed,
      p, i, d);

  public SwerveModule backLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftDriveEncoderReversed,
      DriveConstants.kBackLeftTurningEncoderReversed,
      DriveConstants.kRearLeftEncoderCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.kBackLeftAbsoluteEncoderReversed,
      p, i, d);

  public SwerveModule backRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightDriveEncoderReversed,
      DriveConstants.kBackRightTurningEncoderReversed,
      DriveConstants.kRearRightEncoderCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      DriveConstants.kBackRightAbsoluteEncoderReversed,
      p, i, d);

  // The gru
  private final AHRS gru = new AHRS(NavXComType.kMXP_SPI);

  // Creating odometry object from the kinematics object and the initial wheel positions.
  // Here, our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing the opposing alliance wall.
  public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    this.getRotation2d(),
    new SwerveModulePosition[] {
      frontLeft.getDrivePositionModule(), frontRight.getDrivePositionModule(),
      backLeft.getDrivePositionModule(), backRight.getDrivePositionModule()
    },
    new Pose2d(0, 0, this.getRotation2d())
  );

  public void resetOdometry(Pose2d newPose) {
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      this.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getDrivePositionModule(), frontRight.getDrivePositionModule(),
        backLeft.getDrivePositionModule(), backRight.getDrivePositionModule()
      },
      newPose
    );
  }

  public void resetOdometry() {
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      this.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getDrivePositionModule(), frontRight.getDrivePositionModule(),
        backLeft.getDrivePositionModule(), backRight.getDrivePositionModule()
      },
      new Pose2d()
    );
  }

  // Util for handling turn to theta instructions
  public final TurnThetaHelper thetaHelper = new TurnThetaHelper(getYaw());

  @Override
  public void periodic() {
    // Update the pose
    m_currentDisplacement = updateOdometry();
    SmartDashboard.putData("Gru", gru);
  }

  // Gru data shenanigans
  public void resetPose() {
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    gru.reset();
    gru.resetDisplacement();
    setOffset(pose);
  }

  public void setOffset(Pose2d pose) {
    gru.setAngleAdjustment(pose.getRotation().getDegrees());
    offsetX = pose.getX();
    offsetY = pose.getY();
  }

  public double getHeading() {
    return Math.IEEEremainder(gru.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Rotation2d getInvertedRotation2d() {
    return Rotation2d.fromDegrees(-getHeading());
  }

  public double getYaw() {
    return -gru.getYaw();
  }

  public double getRoll() {
    return gru.getRoll();
  }

  public double getPitch() {
    return gru.getPitch();
  }

  public double getX() {
    return gru.getDisplacementX() + offsetX;
  }

  public double getY() {
    return gru.getDisplacementY() + offsetY;
  }

  public double getXSpeed() {
    return gru.getVelocityX();
  }

  public double getYSpeed() {
    return gru.getVelocityY();
  }

  public double getTurnSpeed() {
    return gru.getRate() / 180 * Math.PI;
  }

  public Pose2d getGruPose() {
    return new Pose2d(getX(), getY(), getRotation2d());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(getXSpeed(), getYSpeed(), getTurnSpeed());
  }

  public Pose2d updateOdometry() {
    m_odometry.update(this.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getDrivePositionModule(), frontRight.getDrivePositionModule(),
        backLeft.getDrivePositionModule(), backRight.getDrivePositionModule()
      }
    );
    m_field.setRobotPose(getOdomentryPose());
    return getOdomentryPose();
  }

  public Pose2d getOdomentryPose() {
    return m_odometry.getPoseMeters();
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
  }

  public void driveAutoRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)));
  }

  public void driveAutoRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)));
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * @param desiredStates The states the modules should move toward. In order,
   *                      front left, front right, back left, back right.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        DriveConstants.kFasterTeleMaxMetersPerSec);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void testMotor() {
    SmartDashboard.putNumber("POV", Controller.getPOV());

    if (Controller.getPOV() > -1) {
      int pov = Controller.getPOV();
      frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
    }

    double speedMultiplyer = Controller.getR1Button() ? .3 : 1;

    SwerveModule module = null;

    if (Controller.getTriangleButton())
      module = frontRight;
    else if (Controller.getSquareButton())
      module = frontLeft;
    else if (Controller.getCircleButton())
      module = backRight;
    else if (Controller.getCrossButton())
      module = backLeft;

    if (module == null) {
      frontLeft.setDesiredState(
          new SwerveModuleState(
              Controller.getLeftY() * speedMultiplyer,
              Rotation2d.fromRotations(.5 * Controller.getLeftX())));
      frontRight.setDesiredState(
          new SwerveModuleState(
              Controller.getLeftY() * speedMultiplyer,
              Rotation2d.fromRotations(.5 * Controller.getLeftX())));
      backLeft.setDesiredState(
          new SwerveModuleState(
              Controller.getLeftY() * speedMultiplyer,
              Rotation2d.fromRotations(.5 * Controller.getLeftX())));
      backRight.setDesiredState(
          new SwerveModuleState(
              Controller.getLeftY() * speedMultiplyer,
              Rotation2d.fromRotations(.5 * Controller.getLeftX())));
      return;
    }

    frontLeft.setDesiredState(new SwerveModuleState());
    frontRight.setDesiredState(new SwerveModuleState());
    backLeft.setDesiredState(new SwerveModuleState());
    backRight.setDesiredState(new SwerveModuleState());

    module.setDesiredState(
        new SwerveModuleState(
            Controller.getLeftY() * speedMultiplyer,
            Rotation2d.fromRotations(.5 * Controller.getLeftX())));
  }
}
