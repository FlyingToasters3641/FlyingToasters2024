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
        Logger.recordOutput("Elevator/ElevatorHeight", inputs.currentElevatorHeight);
    }

    public void setHeight(double position){
        io.setHeight(position);
    }

    public void setClimber(double position){
        io.setClimber(position);
    }




}
