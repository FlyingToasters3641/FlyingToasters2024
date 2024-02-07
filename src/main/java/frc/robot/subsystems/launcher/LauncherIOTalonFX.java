package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class LauncherIOTalonFX implements LauncherIO {
    private static final String CANbusName = "Lucas";
    public static final TalonFX topFlywheelTalonFX = new TalonFX(31, CANbusName);
    public static final TalonFX bottomFlywheelTalonFX = new TalonFX(32, CANbusName);
    public static final TalonFX launcherRollerTalonFX = new TalonFX(33, CANbusName);
    public static final TalonFX launcherPitchTalonFX = new TalonFX(34, CANbusName);
    public static final CANcoder launcherPitchCANCoder = new CANcoder(35, CANbusName);

    /* Start at velocity 0, no feed forward, use slot 0 */
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

    public static final double PIVOT_RATIO = 22.5;//12/48 -> 16/24 -> 12/45
    public static final double SENSOR_RATIO = 6.0;//12/48 -> 16/24
    private final double absoluteEncoderOffset = 0.0;//need to calibrate!

    public LauncherIOTalonFX() {
        bottomFlywheelTalonFX.setInverted(true);
        topFlywheelTalonFX.setInverted(true);
        launcherRollerTalonFX.setInverted(true);

        topFlywheelTalonFX.setControl(new Follower(bottomFlywheelTalonFX.getDeviceID(), false));

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kV = 0.15;
        flywheelConfig.Slot0.kA = 0.80;

        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 80;

        topFlywheelTalonFX.getConfigurator().apply(flywheelConfig);

        TalonFXConfiguration pitchConfig = new TalonFXConfiguration();

        pitchConfig.Feedback.FeedbackRemoteSensorID = launcherPitchCANCoder.getDeviceID();
        pitchConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pitchConfig.Feedback.SensorToMechanismRatio = PIVOT_RATIO;
        pitchConfig.Feedback.RotorToSensorRatio = SENSOR_RATIO;

        pitchConfig.Slot0.kV = 0.15;
        pitchConfig.Slot0.kA = 0.30;
        pitchConfig.Slot0.kP = 0.10;
        pitchConfig.Slot0.kD = 0.0;

        launcherPitchTalonFX.getConfigurator().apply(pitchConfig);

        CANcoderConfiguration magConfig = new CANcoderConfiguration();

        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

        launcherPitchCANCoder.getConfigurator().apply(magConfig);

    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {

    }

    @Override
    public void setPosition(double position) {
        
    }

    @Override
    public void setTopFlywheelVoltage(double volts, double acceleration, double feedforward) {
        topFlywheelTalonFX.setControl(new VelocityVoltage(volts, acceleration, true, feedforward, 0, false, false, false));
    }

    @Override 
    public void setTopFlywheelRollers(double volts) {
        topFlywheelTalonFX.set(volts);
    }

    @Override
    public void setBottomFlywheelVoltage(double volts, double acceleration, double feedforward) {
        bottomFlywheelTalonFX.setControl(new VelocityVoltage(volts, acceleration, true, feedforward, 0, false, false, false));
    }
    
    @Override 
    public void setBottomFlywheelRollers(double volts) {
        bottomFlywheelTalonFX.set(volts);
    }


    @Override
    public void setFeederVoltage(double speed) {
        launcherRollerTalonFX.set(speed);
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        topFlywheelTalonFX.setControl(m_torqueVelocity.withVelocity(rpm));
    }
}