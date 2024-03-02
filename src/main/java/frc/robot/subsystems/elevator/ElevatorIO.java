package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double currentElevatorHeight = 0.0;
    }

    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setBrakeMode(boolean leaderBrake, boolean followerBrake) {}

    public default void setHeight(double position) {}

    public default void getPosition() {}



    
}
