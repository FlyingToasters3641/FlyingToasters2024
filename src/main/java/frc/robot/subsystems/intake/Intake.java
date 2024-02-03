package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    /* 
    private boolean isFrontRunning = false;
    private boolean isRearRunning = false;
    private final SimpleMotorFeedforward intakeFeedforward;
    private final PIDController frontFeedback;
    private final PIDController rearFeedback;*/
    IntakeIOTalonFX IntakeTalonFX = new IntakeIOTalonFX();
    public Intake(IntakeIO io) {
        this.io = io;
        io.setBrakeMode(false, false);
    }
        
    

    @Override
    public void periodic(){

      io.updateInputs(inputs);

      io.setFrontVoltage(inputs.frontIntakeAppliedVolts);
    }

    public void startFrontRollers(){
        io.setFrontVoltage(12.0);
    }

    public void reverseFrontRollers(){
        io.setFrontVoltage(12.0);
        IntakeTalonFX.frontTalonFX.setInverted(false); 
    }
    public void stopFrontRollers(){
        io.setFrontVoltage(0);
    }

    public void runFrontRollers(DoubleSupplier axis){
        io.setFrontSpeed(axis.getAsDouble());
    }
        
}
