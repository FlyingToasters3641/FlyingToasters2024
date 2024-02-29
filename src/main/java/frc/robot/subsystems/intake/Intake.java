package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
   
    IntakeIOTalonFX IntakeTalonFX = new IntakeIOTalonFX();

    public Intake(IntakeIO io) {
        this.io = io;
        io.setBrakeMode(false, false);
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);

    }

    public void reverseFrontRollers(DoubleSupplier axis) {
        io.setFrontSpeed(axis.getAsDouble());
    }

    public void runFrontRollers(DoubleSupplier axis) {
        io.setFrontSpeed(axis.getAsDouble());
    }

    public void reverseRearRollers(DoubleSupplier axis) {
        io.setRearSpeed(axis.getAsDouble());
    }

    public void runRearRollers(double value) {
        io.setRearSpeed(value);
    }
    
    public void runRear() {
        io.setRearSpeed(.5);
    }

    public void runFront() {
        io.setFrontSpeed(.5);
    }

    public boolean frontCurrentSpike() {
        return (IntakeTalonFX.frontTalonFX.getSupplyCurrent().getValue() > 2.0);
    }

    public void stopFront() {
        io.stopFront();
    }

    public void stopRear() {
        io.stopRear();
    }
    
  public boolean getIntakeNote(){
    return inputs.note;
  }
}
