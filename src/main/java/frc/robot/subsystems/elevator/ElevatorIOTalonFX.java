package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
    private static final String CANbusName = "Lucas";
    public static final TalonFX leaderTalonFX = new TalonFX(1, CANbusName);
    public static final TalonFX followTalonFX = new TalonFX(2, CANbusName);

    public static final CANcoder leaderEncoder = new CANcoder(3, CANbusName);

    private Follower followerControl;

    public ElevatorIOTalonFX() {
        followerControl = new Follower(0, false);
        followTalonFX.setControl(followerControl);

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

        leaderTalonFX.getConfigurator().apply(talonFXConfigs);

        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        leaderTalonFX.setControl(m_request.withPosition(100));

        leaderTalonFX.getConfigurator().apply(talonFXConfigs);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

    }

    @Override
    public void setPosition(double position) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(position);
        
        leaderTalonFX.setControl(m_request);
    }
}