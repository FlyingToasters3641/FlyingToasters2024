package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.launcher.Launcher;

public class ElevatorCommands {
    
    private ElevatorCommands() {}
    
    public static Command climb(Elevator m_elevator, RobotSystem m_system){
        return Commands.run(() -> m_system.setGoalState(SystemState.CLIMB_RETRACT)).until(() -> m_elevator.atThreshold() == true).andThen(Commands.runOnce(() -> m_system.setGoalState(SystemState.CLIMB_LOCK)));
    }

    public static ConditionalCommand unlockElevator(Elevator m_elevator, Launcher m_launcher, RobotSystem m_system) {
        return new ConditionalCommand(declimb(m_elevator, m_system), Commands.runOnce(() -> m_system.setGoalState(SystemState.CLIMB_LOCK)),() -> (m_elevator.atThreshold() == true && m_launcher.atThreshold() == true));
    }

    public static Command declimb(Elevator m_Elevator, RobotSystem m_system){
        return Commands.runOnce(() -> m_system.setGoalState(SystemState.CLIMB_RETRACT)).andThen(Commands.runOnce(() -> m_system.setGoalState(SystemState.CLIMB_EXTEND)));
    }
    
}
