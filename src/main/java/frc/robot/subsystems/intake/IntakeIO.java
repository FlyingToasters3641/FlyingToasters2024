package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

    public boolean frontNote = false;
    public boolean rearNote = false;
        
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setBrakeMode(boolean frontBrake, boolean rearBrake) {}

    public default void setFrontSpeed(double speed){}
    
    public default void setRearSpeed(double speed){}

    public default void stopFront(){}

    public default void stopRear(){}

    public default void getFrontNote(){}

}
