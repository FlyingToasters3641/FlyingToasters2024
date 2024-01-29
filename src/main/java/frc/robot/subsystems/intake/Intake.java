package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private boolean isFrontRunning = false;
    private boolean isRearRunning = false;


    public Intake(IntakeIO io) {
        this.io = io;

        io.setBrakeMode(false, false);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }
}
