package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AlignToTargetCommand extends SwerveDriveCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // Required so camera setup is completed
    private final LimelightSubsystem m_limelight_subsystem;

    private static double driveXd = 0;

    private static double driveYd = 0;
    private static double driveThetad = 0;

    ShuffleboardTab m_tab = Shuffleboard.getTab("Limelight");

    GenericEntry p, i, d;

    private PIDController turningPID;

    PS4Controller cont;


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

    public AlignToTargetCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem, PS4Controller cont) {
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

        this.cont = cont;

        p = m_tab.add("p", 1).getEntry();
        i = m_tab.add("i", 0).getEntry();
        d = m_tab.add("d", 0).getEntry();
        
        turningPID = new PIDController(p.getDouble(1), i.getDouble(0), d.getDouble(0));

        System.out.println("AAAAAAA");

        m_limelight_subsystem = limelightSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_limelight_subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose3d posFromTag = LimelightHelpers.getBotPose3d_TargetSpace("");
        
        // double tagId = LimelightHelpers.getFiducialID("");

        double yaw = posFromTag.getRotation().getY(); // Returns the yaw even though it says pitch
        double x = posFromTag.getX();

        if (cont.getCircleButton())
          turningPID = new PIDController(p.getDouble(1), i.getDouble(0), d.getDouble(0));

        double turnSpeed = Math.sin(yaw);
        // Take the smaller speed depending on direction
        double xSpeed = -MathUtil.clamp(x,-1,1);
        // Z in 3d space corrosponds to the Y for the motor
        double ySpeed = Math.min(posFromTag.getZ(), 1);


        //super.setDriveSpeeds(xSpeed, ySpeed, turnSpeed, false);
        //driveXd = xSpeed;
        //driveYd = ySpeed;

        
        driveThetad = turningPID.calculate(yaw, 0);
        super.execute();

    }
}
