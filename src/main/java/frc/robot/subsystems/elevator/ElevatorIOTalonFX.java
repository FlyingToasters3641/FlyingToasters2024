package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.util.Units;

public class ElevatorIOTalonFX implements ElevatorIO{
    
    private static final String CANbusName = "Lucas";
    public static final TalonFX leftTalonFX = new TalonFX(37, CANbusName);
    public static final TalonFX rightTalonFX = new TalonFX(36, CANbusName);
    public static final CANcoder elevatorCANCoder = new CANcoder(40, CANbusName);

    public static double setpoint = 0.0;

    public static final double absoluteEncoderOffset = -0.834; //Calibrate

    public static final double ELEVATOR_RATIO = 6.34375;
    public static final double ENCODER_RATIO = 1;

    public double lastPosition = 0.0;
    public double elevatorThreshold = 0.5;

    public ElevatorIOTalonFX(){
        leftTalonFX.setInverted(false);
        rightTalonFX.setInverted(false);

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.Slot0.kP = 500.0;
        elevatorConfig.Slot0.kD = 30.0;

        elevatorConfig.Slot1.kP = 10.0;


        elevatorConfig.Feedback.FeedbackRemoteSensorID = elevatorCANCoder.getDeviceID();
        elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        elevatorConfig.Feedback.SensorToMechanismRatio = ENCODER_RATIO;
        elevatorConfig.Feedback.RotorToSensorRatio = ELEVATOR_RATIO;

        elevatorConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80)
                .withMotionMagicAcceleration(50);

        leftTalonFX.getConfigurator().apply(elevatorConfig);

        rightTalonFX.setControl(new Follower(leftTalonFX.getDeviceID(), true));

        CANcoderConfiguration magConfig = new CANcoderConfiguration();

        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

        elevatorCANCoder.getConfigurator().apply(magConfig);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionSetpointInches = setpoint;
        inputs.position = -elevatorCANCoder.getPosition().getValueAsDouble();
        Logger.recordOutput("Elevator/absolute", elevatorCANCoder.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/leftMotorPos", leftTalonFX.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/rightMotorPos", rightTalonFX.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/Voltage", leftTalonFX.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/Setpoint", setpoint);
    }

    @Override
    public void setPosition(double position){
        setpoint = position;
        leftTalonFX.setControl(new MotionMagicTorqueCurrentFOC(-position).withSlot(0));

    }

    @Override
    public boolean atThreshold() {

        double currentAngle = -elevatorCANCoder.getPosition().getValueAsDouble();
        
        if (currentAngle >= (setpoint - elevatorThreshold) 
        && currentAngle <= (setpoint + elevatorThreshold)) {
            return true;
        } else {
            return false;
        }
    }



}
