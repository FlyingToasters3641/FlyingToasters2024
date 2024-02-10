package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation3d;

public interface LauncherIO {
    @AutoLog
    public static class LauncherIOInputs {
        public double topFlywheelAppliedVolts = 0.0;

        public double bottomFlywheelAppliedVolts = 0.0;

        public double feederAppliedVolts = 0.0;

        public double wristAppliedVolts = 0.0;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}

    public default void setTopFlywheelVoltage(double volts, double acceleration, double feedforward) {}

    public default void setTopFlywheelRollers(double speed) {}

    public default void setBottomFlywheelVoltage(double volts, double acceleration, double feedforward) {}

    public default void setBottomFlywheelRollers(double speeds) {}
    
    public default void setFeederVoltage(double speed) {}

    public default void setBrakeMode(boolean topFlywheelBrake, boolean bottomFlywheelBrake, boolean feederBrake, boolean wristBrake) {}

    public default void setPosition(double position) {}

    public default void stopFlywheelTop() {}

    public default void stopBottomFlywheel() {}

    public default Translation3d calcTrajectory(Translation3d robotTrajectory, Translation3d shooterTrajectory){
        return null;
    }

}

