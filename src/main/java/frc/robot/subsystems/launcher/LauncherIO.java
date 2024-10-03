package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
    @AutoLog
    public static class LauncherIOInputs {
        public double topFlywheelAppliedVolts = 0.0;

        public double bottomFlywheelAppliedVolts = 0.0;

        public double feederAppliedVolts = 0.0;

        public double wristAppliedVolts = 0.0;
                
        public double launcherAngleRads = 0.0;

        public double launcherPositionDegrees = 0.0;

        public double launcherPosition = 0.0;

        public double pitchMotorSensorDegrees = 0.0;

        public double angleSetpointDegrees = 0.0;

        public double flywheelVelocity = 0.0;
        
        public boolean note = false;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}
    
    public default void setFeederVoltage(double speed) {}

    public default void setBrakeMode(boolean topFlywheelBrake, boolean bottomFlywheelBrake, boolean feederBrake, boolean wristBrake) {}

    public default void setAngleSetpoint(double angle) {}

    public default void setAngle(double angle) {}

    public default void stop() {}

    public default void setFlywheelVelocity(double rpm) {}

    public default void setTopFlywheelVelocity(double rpm) {}

    public default void setBottomFlywheelVelocity(double rpm) {}

    public default void getAngle() {}

    public default void getLauncherNote() {}

    public default boolean atShooterThreshold() {return true;}

    public default boolean atThreshold() {return true;}

    public default void setBlower(boolean Powered) {};
}

