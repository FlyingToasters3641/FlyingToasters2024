package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final double threshold = 0.1;

    public Elevator(ElevatorIO io){
        this.io = io;
        
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public boolean atThreshold(){
        return (Math.abs(inputs.positionSetpointInches - inputs.position) <= threshold);
    }
    
}