package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double frontIntakeAppliedVolts = 0.0;

        public double rearIntkakeAppliedVolts = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setFrontVoltage(double volts) {}

    public default void setRearVoltage(double volts) {}

    public default void setBrakeMode(boolean frontBrake, boolean rearBrake) {}

    public default void setFrontSpeed(double speed){}
    
    public default void setRearSpeed(double speed){}

}
