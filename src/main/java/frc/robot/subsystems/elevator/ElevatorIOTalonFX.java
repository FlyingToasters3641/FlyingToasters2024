package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final String CANbusName = "Lucas";
    public final TalonFX leaderTalonFX = new TalonFX(2, CANbusName);
    public final TalonFX followerTalonFX = new TalonFX(1, CANbusName);
    public final CANcoder leaderEncoder = new CANcoder(3, CANbusName);

    public double elevatorPosition = 0.0;
    public double elevatorAcceleration = 0.0;

    private MotionMagicVelocityVoltage m_VelocityVoltage = new MotionMagicVelocityVoltage(0.0);

    public ElevatorIOTalonFX() {

        followerTalonFX.setControl(new Follower(leaderTalonFX.getDeviceID(), false));

        // in init function
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        leaderTalonFX.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.elevatorPosition = leaderEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.elevatorAcceleration = elevatorAcceleration;
    }

    @Override
    public void setPosition(double position) {

        leaderTalonFX.setControl(m_VelocityVoltage.withVelocity(position));
        elevatorPosition = position;
    }

    @Override
    public void setPosition(double position, double acceleration) {

        m_VelocityVoltage.Acceleration = acceleration;
        leaderTalonFX.setControl(m_VelocityVoltage.withVelocity(position));

        elevatorPosition = position;
    }

    @Override
    public void setBrakeMode(boolean leaderFlywheelBrake, boolean followerFlywheelBrake) {
        leaderTalonFX.setNeutralMode(leaderFlywheelBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerTalonFX.setNeutralMode(followerFlywheelBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

}
