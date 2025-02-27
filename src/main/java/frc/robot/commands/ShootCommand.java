package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand (
        ElevatorSubsystem m_elevatorSubsystem,
        CoralSubsystem m_coralSubsystem
    ) {
        addCommands(
            new ElevatorCommand(m_elevatorSubsystem, () -> 0),
            new CoralCommand(m_coralSubsystem, true)
        );
    }
}