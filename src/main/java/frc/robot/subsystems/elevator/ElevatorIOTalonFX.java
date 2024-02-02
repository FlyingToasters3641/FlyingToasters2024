package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
    private static final String CANbusName = "Lucas";
    public static final TalonFX leaderTalonFX = new TalonFX(1, CANbusName);
    public static final TalonFX followTalonFX = new TalonFX(2, CANbusName);

    public static final CANcoder leaderEncoder = new CANcoder(3,CANbusName);

    
    public ElevatorIOTalonFX() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        
    }

}