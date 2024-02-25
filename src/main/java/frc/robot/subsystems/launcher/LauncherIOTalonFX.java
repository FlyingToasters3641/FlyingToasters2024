package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class LauncherIOTalonFX implements LauncherIO {
    DigitalInput launchSensor = new DigitalInput(0);
    private static final String CANbusName = "Lucas";
    public static final TalonFX topFlywheelTalonFX = new TalonFX(31, CANbusName);
    public static final TalonFX bottomFlywheelTalonFX = new TalonFX(32, CANbusName);
    public static final TalonFX launcherRollerTalonFX = new TalonFX(33, CANbusName);
    public static final TalonFX launcherPitchTalonFX = new TalonFX(34, CANbusName);
    public static final CANcoder launcherPitchCANCoder = new CANcoder(35, CANbusName);

    

    /* Start at velocity 0, no feed forward, use slot 0 */
    private final VelocityVoltage m_Velocity = new VelocityVoltage(0.0);

    public static final double PIVOT_RATIO = 22.5;// 12/48 -> 16/24 -> 12/45
    private final double absoluteEncoderOffset = 0.2855;// need to calibrate!

    private double launcherSetpointDegrees = 0.0;

    public LauncherIOTalonFX() {
        bottomFlywheelTalonFX.setInverted(true);
        bottomFlywheelTalonFX.setInverted(true);
        launcherRollerTalonFX.setInverted(true);
        launcherPitchTalonFX.setInverted(false);

        topFlywheelTalonFX.setControl(new Follower(bottomFlywheelTalonFX.getDeviceID(), false));

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kV = 0.15;
        flywheelConfig.Slot0.kA = 0.80;

        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 80;

        topFlywheelTalonFX.getConfigurator().apply(flywheelConfig);
        bottomFlywheelTalonFX.getConfigurator().apply(flywheelConfig);

        TalonFXConfiguration pitchConfig = new TalonFXConfiguration();

        pitchConfig.Feedback.FeedbackRemoteSensorID = launcherPitchCANCoder.getDeviceID();
        pitchConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pitchConfig.Feedback.SensorToMechanismRatio = 1;
        pitchConfig.Feedback.RotorToSensorRatio = PIVOT_RATIO;

        //Esitmated Values from Recalc
        pitchConfig.Slot0.kG = -30.00;
        pitchConfig.Slot0.kV = 2.79;
        pitchConfig.Slot0.kA = 0.04;

        pitchConfig.Slot0.kP = 1000;
        pitchConfig.Slot0.kD = 20;

        pitchConfig.Slot1.kG = -20.00;
        pitchConfig.Slot1.kV = 2.79;
        pitchConfig.Slot1.kA = 0.04;

        pitchConfig.Slot1.kP = 100;
        pitchConfig.Slot1.kD = 2;
        
        pitchConfig.Slot2.kG = -7.00;
        pitchConfig.Slot2.kV = 2.79;
        pitchConfig.Slot2.kA = 0.04;

        pitchConfig.Slot2.kP = 300;
        pitchConfig.Slot2.kD = 10;
        

        pitchConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                        Units.radiansToRotations(LauncherConstants.profileConstraints.crusieVelocityRadPerSec()))
                .withMotionMagicAcceleration(LauncherConstants.profileConstraints.accelerationRadPerSec2());

        launcherPitchTalonFX.getConfigurator().apply(pitchConfig);

        CANcoderConfiguration magConfig = new CANcoderConfiguration();

        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

        launcherPitchCANCoder.getConfigurator().apply(magConfig);

    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        inputs.launcherPosition = (launcherPitchCANCoder.getAbsolutePosition().getValue());
        inputs.launcherPositionDegrees = (Units.rotationsToDegrees(launcherPitchCANCoder.getPosition().getValue()));
        inputs.pitchMotorSensorDegrees = Units.rotationsToDegrees(launcherPitchTalonFX.getPosition().getValue());
        inputs.angleSetpointDegrees = launcherSetpointDegrees;
        inputs.note = launchSensor.get();
        Logger.recordOutput("Launcher/Sensor", launchSensor.get());
    }

    @Override
    public void setBrakeMode(boolean topFlywheelBrake, boolean bottomFlywheelBrake, boolean feederBrake,
            boolean wristBrake) {
        topFlywheelTalonFX.setNeutralMode(topFlywheelBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        bottomFlywheelTalonFX.setNeutralMode(bottomFlywheelBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        launcherRollerTalonFX.setNeutralMode(feederBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        launcherPitchTalonFX.setNeutralMode(wristBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setAngleSetpoint(double setpointDegrees) {
        if (setpointDegrees > 20.0 && setpointDegrees <= 100){
            launcherPitchTalonFX.setControl(new MotionMagicTorqueCurrentFOC(Units.degreesToRotations(-setpointDegrees)).withSlot(0));
        } else if (setpointDegrees > 100) {
            launcherPitchTalonFX.setControl(new MotionMagicTorqueCurrentFOC(Units.degreesToRotations(-setpointDegrees)).withSlot(2));
        }
        else{
            launcherPitchTalonFX.setControl(new MotionMagicTorqueCurrentFOC(Units.degreesToRotations(-setpointDegrees)).withSlot(1));
        }
        launcherSetpointDegrees = setpointDegrees;
    }

    @Override
    public void setAngle(double angleRads) {
        launcherPitchTalonFX.setPosition(Units.radiansToRotations(angleRads));
    }

    @Override
    public void setFeederVoltage(double speed) {
        launcherRollerTalonFX.set(speed);
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        topFlywheelTalonFX.setControl(m_Velocity.withVelocity(-rpm));
        bottomFlywheelTalonFX.setControl(m_Velocity.withVelocity(-rpm));
    }

    @Override
    public void stop() {
        setFlywheelVelocity(0.0);
        setFeederVoltage(0.0);
    }

}