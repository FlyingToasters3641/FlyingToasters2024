package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
    private final String CANbusName = "Lucas";
    public final TalonFX frontTalonFX = new TalonFX(2, CANbusName);
    public final TalonFX rearTalonFX = new TalonFX(1, CANbusName);

    public static double frontTalonVoltage;

    public IntakeIOTalonFX() {
       frontTalonFX.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
    }

    @Override
    public void setFrontVoltage(double Volts){
        frontTalonFX.setVoltage(Volts);
    }

}
