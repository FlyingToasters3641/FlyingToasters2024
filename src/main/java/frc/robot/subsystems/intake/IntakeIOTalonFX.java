package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
    private final String CANbusName = "Lucas";
    public final TalonFX frontTalonFX = new TalonFX(1, CANbusName);
    public final TalonFX rearTalonFX = new TalonFX(2, CANbusName);

    public IntakeIOTalonFX() {
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        
    }

}
