package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  PS4Controller Controller;

  public ShuffleboardTab m_tab = Shuffleboard.getTab("swerve");

  public GenericEntry p = m_tab.add("p", ModuleConstants.kTurningP).getEntry();
  public GenericEntry i = m_tab.add("i", ModuleConstants.kTurningI).getEntry();
  public GenericEntry d = m_tab.add("d", ModuleConstants.kTurningD).getEntry();

  public double offsetX = 0;
  public double offsetY = 0;

  public SwerveSubsystem(PS4Controller controller) {

    //Delay to allow navx to boot up
    new Thread(() -> {
      try {
        Thread.sleep(DriveConstants.kBootupDelay);
        resetPose();
      } catch (Exception e) {}
    }).start();

    //Shuffle bored
    m_tab = Shuffleboard.getTab("Swerve");
    m_tab.addNumber("Heading", this::getHeading);
    m_tab.addNumber("Yaw", this::getYaw);
    m_tab.addNumber("Roll", this::getRoll);
    m_tab.addNumber("Pitch", this::getPitch);
    m_tab.addNumber("X", this::getX);
    m_tab.addNumber("Y", this::getY);
    m_tab.addNumber("X Offset", () -> offsetX);
    m_tab.addNumber("Y Offset", () -> offsetY);
    m_tab.addBoolean("NavX isConnected", gru::isConnected);
    m_tab.addBoolean("NavX isCalibrating", gru::isCalibrating);

    Controller = controller;

    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
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
    p,i,d
  );

  public SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightEncoderCanId,
    DriveConstants.kFrontRightChassisAngularOffset,
    DriveConstants.kFrontRightAbsoluteEncoderReversed,
    p,i,d
  );

  public SwerveModule backLeft = new SwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kRearLeftEncoderCanId,
    DriveConstants.kBackLeftChassisAngularOffset,
    DriveConstants.kBackLeftAbsoluteEncoderReversed,
    p,i,d
  );

  public SwerveModule backRight = new SwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kRearRightEncoderCanId,
    DriveConstants.kBackRightChassisAngularOffset,
    DriveConstants.kBackRightAbsoluteEncoderReversed,
    p,i,d
  );

  // The gru
  private final AHRS gru = new AHRS(NavXComType.kMXP_SPI);

  // Gru data shenanigans
  public void resetPose() {
    gru.reset();
    gru.resetDisplacement();
    setOffset(new Pose2d());
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
  public double getHeading() {return Math.IEEEremainder(gru.getAngle(), 360);}
  public Rotation2d getRotation2d() {return Rotation2d.fromDegrees(getHeading());}
  public double getYaw() {return Math.IEEEremainder(gru.getYaw(), 360);}
  public double getRoll() {return Math.IEEEremainder(gru.getRoll(), 360);}
  public double getPitch() {return Math.IEEEremainder(gru.getPitch(), 360);}
  public double getX() {return gru.getDisplacementX() + offsetX;}
  public double getY() {return gru.getDisplacementY() + offsetY;}
  public double getXSpeed() {return gru.getVelocityX();}
  public double getYSpeed() {return gru.getVelocityY();}
  public double getTurnSpeed() {return gru.getRate()/180*Math.PI;}
  public Pose2d getPose() {return new Pose2d(getX(), getY(), getRotation2d());}

  public ChassisSpeeds getRobotRelativeSpeeds() {return new ChassisSpeeds(getXSpeed(), getYSpeed(), getTurnSpeed());}
  public void driveRobotRelative(ChassisSpeeds speeds) {
      setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
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
      DriveConstants.kFasterTeleMaxMetersPerSec
    );
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void testMotor() {
    System.out.println(Controller.getPOV());
    m_tab.addInteger("POV", Controller::getPOV);

    if (Controller.getPOV() > -1) {
      int pov = Controller.getPOV();
      frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
    }

    double speedMultiplyer = Controller.getR1Button() ? .3 : 1;

    SwerveModule module = null;

    if (Controller.getTriangleButton()) module = frontRight;
    else if (Controller.getSquareButton()) module = frontLeft;
    else if (Controller.getCircleButton()) module = backRight;
    else if (Controller.getCrossButton()) module = backLeft;

    if (module == null) {
      frontLeft.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      frontRight.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      backLeft.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      backRight.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      return;
    }

    frontLeft.setDesiredState(new SwerveModuleState());
    frontRight.setDesiredState(new SwerveModuleState());
    backLeft.setDesiredState(new SwerveModuleState());
    backRight.setDesiredState(new SwerveModuleState());

    module.setDesiredState(
      new SwerveModuleState(
        Controller.getLeftY() * speedMultiplyer,
        Rotation2d.fromRotations(.5 * Controller.getLeftX())
      )
    );
  }
}
