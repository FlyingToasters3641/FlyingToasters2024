package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final String CANbusName = "Lucas";
    private final TalonFX leaderTalonFX = new TalonFX(1, CANbusName);
    private final TalonFX followTalonFX = new TalonFX(2, CANbusName);

    public ElevatorIOTalonFX() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        
    }

}