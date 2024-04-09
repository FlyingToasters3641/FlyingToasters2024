package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeIOTalonFXComp implements IntakeIO {
    private final String CANbusName = "Lucas";
    public final TalonFX frontTalonFX = new TalonFX(2, CANbusName);
    public final TalonFX rearTalonFX = new TalonFX(1, CANbusName);

    
    public final CANSparkMax externalCANSpark = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushed);

    public final DigitalInput frontSensor = new DigitalInput(2);
    public final DigitalInput rearSensor = new DigitalInput(1);
    

    public static double frontTalonVoltage;

    public IntakeIOTalonFXComp() {
       frontTalonFX.setInverted(true);
       externalCANSpark.setInverted(false);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        
        inputs.frontNote = frontSensor.get();
        inputs.rearNote = rearSensor.get();

        Logger.recordOutput("Intake/frontSensor", frontSensor.get());
        Logger.recordOutput("Intake/rearSensor", rearSensor.get());
        SmartDashboard.putBoolean("IntakeFront", frontSensor.get());
        SmartDashboard.putBoolean("IntakeRear", rearSensor.get());

    }


    @Override
    public void setFrontSpeed(double speed){
        frontTalonFX.set(speed);
    }

    
    @Override
    public void setRearSpeed(double speed){
        rearTalonFX.set(speed);
        double extSpeed = speed;
        if (speed > 0){
            extSpeed = 1.0;
        } else if (speed < 0){
            extSpeed = -1.0;
        }else{
            extSpeed = 0.0;
        }

        if (rearSensor.get() == false) {
            extSpeed = 0.0;
        }
        
        externalCANSpark.set(extSpeed);
    }

    @Override
    public void stopFront(){
        frontTalonFX.set(0.0);
    }

    @Override
    public void stopRear(){
        rearTalonFX.set(0.0);
        externalCANSpark.set(0.0);
    }
}
