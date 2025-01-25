package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.DriveConstants;

public class AlignToTargetCommand extends SwerveDriveCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // Required so camera setup is completed
    private final LimelightSubsystem m_limelight_subsystem;

    private static double driveXd = 0;
    private static double driveYd = 0;
    private static double driveThetad = 0;

    private PS4Controller m_controller;

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
    private static Supplier<Double> driveTheta = new Supplier<Double>() {
      @Override
      public Double get() {
        return driveThetad;
      }
    };

    public AlignToTargetCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem, PS4Controller controller) {
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
          driveTheta,
          false,
          () -> false,
          () -> false,
          () -> -1d,
          () -> 0d,
          () -> 0d
        );

        m_limelight_subsystem = limelightSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_limelight_subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose3d posFromTag = LimelightHelpers.getBotPose3d_TargetSpace("");
        
        System.out.println(posFromTag.getX());
        // double tagId = LimelightHelpers.getFiducialID("");

        double yaw = posFromTag.getRotation().getZ();
        double x = posFromTag.getX();

        double turnSpeed = Math.sin(yaw);
        // Take the smaller speed depending on direction
        double xSpeed = -MathUtil.clamp(x,-1,1);
        // Z in 3d space corrosponds to the Y for the motor
        double ySpeed = Math.min(posFromTag.getZ() * DriveConstants.kAutoSpeedLimit, 1);

        //GET READY TO RUUUUMMMMBBBLLLLEEEEE
        if (yaw > 0) {
          m_controller.setRumble(RumbleType.kLeftRumble, 0.5);
        }
        else if (yaw < 0) {
          m_controller.setRumble(RumbleType.kRightRumble, 0.5);
        }


        // super.setDriveSpeeds(xSpeed, ySpeed, turnSpeed, false);
        driveXd = xSpeed;
        driveYd = ySpeed;
        driveThetad = turnSpeed;
        super.execute();

    }
}
