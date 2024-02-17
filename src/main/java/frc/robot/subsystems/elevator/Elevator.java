package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io){
        this.io = io;
        io.setBrakeMode(false, false);
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.recordOutput("Elevator/AbsoluteElevatorPosition", inputs.elevatorPosition);
        Logger.recordOutput("Elevator/Acceleration", inputs.elevatorAcceleration);
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public void setPosition(double position, double acceleration){
        io.setPosition(position, acceleration);
    }



}
