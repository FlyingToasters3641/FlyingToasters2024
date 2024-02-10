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

        public double velocityRadPerSec = 0.0;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}

    public default void runVolts(double volts) {}
    
    public default void setFeederVoltage(double speed) {}

    public default void setBrakeMode(boolean topFlywheelBrake, boolean bottomFlywheelBrake, boolean feederBrake, boolean wristBrake) {}

    public default void setAngleSetpoint(double angle) {}

    public default void setAngle(double angle) {}

    public default void stop() {}

    public default void setFlywheelVelocity(double rpm) {}

    public default void setPID(double kP,double kI, double kD) {}

    public default void setFF(double kS,double kV, double kA) {}

}

