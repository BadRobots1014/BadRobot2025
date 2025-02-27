package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand (
        ElevatorSubsystem m_elevatorSubsystem,
        CoralSubsystem m_coralSubsystem,
        Supplier<Integer> goalPosition
    ) {
        addCommands(
            new ElevatorCommand(m_elevatorSubsystem, goalPosition).withTimeout(4),
            new CoralCommand(m_coralSubsystem, true).withTimeout(2)
        );
    }
}