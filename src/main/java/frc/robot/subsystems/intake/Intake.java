package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    //Systems and Logging Initialization
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    IntakeIOTalonFX IntakeTalonFX = new IntakeIOTalonFX();

    /**
   * Includes the IO class to allow for logging
   */
    public Intake(IntakeIO io) {
        this.io = io;
        io.setBrakeMode(false, false);
    }

    //Constantly updates inputs
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /**
     * Spins the front rollers inward
     */
    public void reverseFrontRollers(DoubleSupplier axis) {
        io.setFrontSpeed(axis.getAsDouble());
    }

    /**
     * Spins the front rollers outward
     */
    public void runFrontRollers(DoubleSupplier axis) {
        io.setFrontSpeed(axis.getAsDouble());
    }

    /**
     * Spins the back rollers inward to the robot
     */
    public void reverseRearRollers(DoubleSupplier axis) {
        io.setRearSpeed(axis.getAsDouble());
    }

    /**
     * Spins the back rollers outward to the robot
     */
    public void runRearRollers(double value) {
        io.setRearSpeed(value);
    }
    
    /**
     * Fixed speed of spinning the back rollers outward
     */
    public void runRear() {
        io.setRearSpeed(.5);
    }

    /**
     * Fixed speed of spinning the front rollers outward
     */
    public void runFront() {
        io.setFrontSpeed(.5);
    }

    /**
     * Fixed speed of spinning the back rollers inward
     */
    public void reverseRear() {
        io.setRearSpeed(-0.5);
    }

    /**
     * Fixed speed of spinning the front rollers inward
     */
    public void reverseFront() {
        io.setFrontSpeed(-0.5);
    }

    /**
     * Fixed speed of spinning the back rollers outward
     */
    public void runRearMax() {
        io.setRearSpeed(1.0);
    }


    /**
     * Returns whether the front rollers are at max speed
     */
    public boolean frontCurrentSpike() {
        return (IntakeTalonFX.frontTalonFX.getSupplyCurrent().getValue() > 2.0);
    }

    /**
     * Stops the Front Intake from running. Sets the Speed to 0
     */
    public void stopFront() {
        io.stopFront();
    }

    /**
     * Stops the Rear Intake from running. Sets the Speed to 0
     */
    public void stopRear() {
        io.stopRear();
    }

    /**
     * Returns whether the Front Intake Sensor received a note. Returns False if it detected a note   
     */
    public boolean getFrontNote() {
        return inputs.frontNote;
    }

    /**
     * Returns whether the Rear Intake Sensor received a note. Returns False if it detected a note     
     */
    public boolean getRearNote() {
        return inputs.rearNote;
    }
}
