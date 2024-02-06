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
    public static final TalonFX topFlywheelTalonFX = new TalonFX(31, CANbusName);
    public static final TalonFX bottomFlywheelTalonFX = new TalonFX(32, CANbusName);



    public LauncherIOTalonFX() {
        bottomFlywheelTalonFX.setInverted(true);
        topFlywheelTalonFX.setInverted(true);
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {

    }

    @Override
    public void setPosition(double position) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

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
    public void setFeederVoltage(double volts) {
        bottomFlywheelTalonFX.setControl(new VelocityVoltage(volts));
    }

    @Override
    public void stopFlywheelTop(){
        topFlywheelTalonFX.set(0.0);
    }

    @Override
    public void stopBottomFlywheel(){
        bottomFlywheelTalonFX.set(0.0);
    }
}