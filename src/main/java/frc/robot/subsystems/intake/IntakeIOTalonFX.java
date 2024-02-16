package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.drive.ModuleIO;

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
