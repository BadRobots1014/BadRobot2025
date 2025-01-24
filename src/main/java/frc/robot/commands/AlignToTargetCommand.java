package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.DriveConstants;

public class AlignToTargetCommand extends SwerveDriveCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // Required so camera setup is completed
    private final LimelightSubsystem m_limelight_subsystem;

    public AlignToTargetCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        super(
          swerveSubsystem,
          
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
        double x = posFromTag.getX() * DriveConstants.kAutoSpeedLimit;

        double turnSpeed = Math.sin(yaw) * DriveConstants.kAutoSpeedLimit;
        // Take the smaller speed depending on direction
        double xSpeed = x > 0 ? Math.min(Math.abs(x), 1) : Math.max(Math.abs(x), -1);
        // Z in 3d space corrosponds to the Y for the motor
        double ySpeed = Math.min(posFromTag.getZ() * DriveConstants.kAutoSpeedLimit, 1);

        super.setDriveSpeeds(xSpeed, ySpeed, turnSpeed, false);
    }
}
