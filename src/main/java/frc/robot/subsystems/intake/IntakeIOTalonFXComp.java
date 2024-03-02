package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;


public class IntakeIOTalonFXComp implements IntakeIO {
    private final String CANbusName = "Lucas";
    public final TalonFX frontTalonFX = new TalonFX(2, CANbusName);
    public final TalonFX rearTalonFX = new TalonFX(1, CANbusName);

    public final DigitalInput frontSensor = new DigitalInput(2);
    public final DigitalInput rearSensor = new DigitalInput(1);
    

    public static double frontTalonVoltage;

    public IntakeIOTalonFXComp() {
       frontTalonFX.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        
        inputs.frontNote = frontSensor.get();
        inputs.rearNote = rearSensor.get();

        Logger.recordOutput("Intake/frontSensor", frontSensor.get());
        Logger.recordOutput("Intake/rearSensor", rearSensor.get());

    }


    @Override
    public void setFrontSpeed(double speed){
        frontTalonFX.set(speed);
    }

    @Override
    public void setRearSpeed(double speed){
        rearTalonFX.set(speed);
    }

    @Override
    public void stopFront(){
        frontTalonFX.set(0.0);;
    }

    @Override
    public void stopRear(){
        rearTalonFX.set(0.0);
    }
}