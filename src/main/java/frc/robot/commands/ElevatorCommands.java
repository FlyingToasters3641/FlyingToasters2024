package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    private ElevatorCommands () {}

    public static Command goToHeight(Elevator m_elevator, double height){
        return Commands.runOnce(() -> {
            m_elevator.setPosition(height);
        });
    }

    
    
}
