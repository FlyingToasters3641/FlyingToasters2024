package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double leaderAppliedVolts = 0.0;

        public double followerAppliedVolts = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setFrontVoltage(double volts) {}

    public default void setRearVoltage(double volts) {}

    public default void setBrakeMode(boolean leaderBrake, boolean followerBrake) {}

    public default void setPosition(double position) {}

}