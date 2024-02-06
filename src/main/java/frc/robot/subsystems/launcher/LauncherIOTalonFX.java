package frc.robot.subsystems.launcher;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class LauncherIOTalonFX implements LauncherIO {
    private static final String CANbusName = "Lucas";
    public static final TalonFX topFlywheelTalonFX = new TalonFX(1, CANbusName);
    public static final TalonFX bottomFlywheelTalonFX = new TalonFX(2, CANbusName);
    public static final TalonFX feederTalonFX = new TalonFX(3, CANbusName);
    public static final TalonFX wristTalonFX = new TalonFX(4, CANbusName);

    public static final CANcoder leaderEncoder = new CANcoder(3, CANbusName);


    public LauncherIOTalonFX() {

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25;
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01;
        slot0Configs.kP = 4.8;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;

        wristTalonFX.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {

    }

    @Override
    public void setPosition(double position) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        wristTalonFX.setControl(m_request);
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
        topFlywheelTalonFX.set(volts);
    }


    @Override
    public void setFeederVoltage(double volts) {
        bottomFlywheelTalonFX.setControl(new VelocityVoltage(volts));
    }


}