package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    private ElevatorCommands () {}

    public static Command goToHeight(Elevator m_elevator, double height){
        return Commands.runOnce(() -> {
            m_elevator.setHeight(height);
        });
    }

    public static Command climb(Elevator m_elevator, double height){
        return Commands.runOnce(() -> {
            m_elevator.setClimber(height);
        });
    }

    //this def don't work
    public static Command climbState(Elevator m_elevator){
        return new SequentialCommandGroup(Commands.runOnce(() -> {
            m_elevator.setClimber(1);
        }),new WaitCommand(1), Commands.runOnce(() -> {
            m_elevator.setClimber(0);
        }));
    }
    
}
