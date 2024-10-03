package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    
    private ElevatorCommands() {}
    
    /**Activates the climb mechanism for the elevator
     * 
     * @param m_elevator Elevator subsystem
     * @param m_system RobotSystem subsystem
     * 
     */
    public static Command climb(Elevator m_elevator, RobotSystem m_system){
        return Commands.run(() -> m_system.setGoalState(SystemState.CLIMB_RETRACT)).until(() -> m_elevator.withinPosition(0.2) == true).andThen(new WaitCommand(0.5)).andThen(Commands.runOnce(() -> m_system.setGoalState(SystemState.CLIMB_LOCK)));
    }
    
    
}
