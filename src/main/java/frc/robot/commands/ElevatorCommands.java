package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorPositions.ElevatorPos;

public class ElevatorCommands {

    private ElevatorCommands() {}

    private Command runElevator(Elevator m_elevator, ElevatorPos elevatorPos, double wristAngle){
        return Commands.run(() -> {m_elevator.setElevatorPosition(elevatorPos.getElevatorPosition());});
    }

   
    

}
