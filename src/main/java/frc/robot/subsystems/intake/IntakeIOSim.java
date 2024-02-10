package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO{

    
    private static final double LOOP_PERIOD_SECS = 0.02;

    private double frontIntakeVolts = 0.0;
    private double rearIntakeVolts = 0.0;

    private final FlywheelSim frontIntake = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0);
    private final FlywheelSim rearIntake = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0);

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        frontIntake.update(LOOP_PERIOD_SECS);
        rearIntake.update(LOOP_PERIOD_SECS);

        inputs.frontIntakeVolts = frontIntakeVolts;
        inputs.rearIntakeVolts = rearIntakeVolts;
    }

    @Override
    public void setFrontSpeed(double speed){
        frontIntakeVolts = speed;
        frontIntake.setInputVoltage(speed);
    }

     @Override
    public void stopFront(){
        setFrontSpeed(0);
        frontIntake.setInputVoltage(0);
    }

    @Override
    public void setRearSpeed(double speed){
      rearIntakeVolts = speed;
      rearIntake.setInputVoltage(speed);
    }

    @Override
    public void stopRear(){
        setRearSpeed(0);
        rearIntake.setInputVoltage(0);
    }
    
}
