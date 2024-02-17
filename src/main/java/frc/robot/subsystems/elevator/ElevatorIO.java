package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPosition = 0.0;
        public double elevatorAcceleration = 0.0;
    }

    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setBrakeMode(boolean leaderBrake, boolean followerBrake) {}

    public default void setPosition(double position) {}

    public default void setPosition(double position, double acceleration) {}

    public default void getPosition() {}



    
}
