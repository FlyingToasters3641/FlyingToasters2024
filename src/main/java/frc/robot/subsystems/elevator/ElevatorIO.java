package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {

        public double positionSetpointInches = 0.0;
        public double position = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void getAngle() {}

    public default boolean atThreshold() {return true;}
    
}
