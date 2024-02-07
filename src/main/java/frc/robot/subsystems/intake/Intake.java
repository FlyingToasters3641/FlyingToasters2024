package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private static boolean isFrontRunning = false;
    private static boolean isRearRunning = false;
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

    }
    public void reverseFrontRollers(DoubleSupplier axis){ 
        io.setFrontSpeed(axis.getAsDouble());
    }
    public void runFrontRollers(DoubleSupplier axis){
        io.setFrontSpeed(axis.getAsDouble());
    }
      public void reverseRearRollers(DoubleSupplier axis){
        io.setRearSpeed(axis.getAsDouble());
    }
    public void runRearRollers(DoubleSupplier axis){
        io.setRearSpeed(axis.getAsDouble());
    }
    public boolean frontCurrentSpike() {
        return (IntakeTalonFX.frontTalonFX.getSupplyCurrent().getValue() > 2.0);
    }

    public void stopFront() {
        io.stopFront();
    }

    public void startFront() {
        io.startFront();
    }



    public void stopRear() {
        io.stopRear();
    }
}
