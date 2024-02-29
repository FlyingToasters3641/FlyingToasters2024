package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.launcher.LauncherConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final String CANbusName = "Lucas";
    public final TalonFX leaderTalonFX = new TalonFX(2, CANbusName);
    public final TalonFX followerTalonFX = new TalonFX(1, CANbusName);
    public final CANcoder leaderCANcoder = new CANcoder(3, CANbusName);


    //Measured in whatever units
    public double currentElevatorHeight = 0.0;

    public double elevatorAcceleration = 0.0;

    

    private MotionMagicVelocityVoltage m_VelocityVoltage = new MotionMagicVelocityVoltage(0.0);

    
    private final double absoluteEncoderOffset = 0.2855;// need to calibrate!

    public ElevatorIOTalonFX() {

        //set motor to follow leader
        followerTalonFX.setControl(new Follower(leaderTalonFX.getDeviceID(), false));

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        //NONE OF THESE ARE TESTED!!!
        // set slot 0 gains (elevator moving up)
        elevatorConfig.Slot0.kG = 0.0;
        elevatorConfig.Slot0.kV = 0.0;
        elevatorConfig.Slot0.kA = 0.0;

        elevatorConfig.Slot0.kP = 0.0;
        elevatorConfig.Slot0.kD = 0.0;

        //set slot 1 gains (elevator moving down)
        elevatorConfig.Slot1.kG = 0.0;
        elevatorConfig.Slot1.kV = 0.0;
        elevatorConfig.Slot1.kA = 0.0;

        elevatorConfig.Slot1.kP = 0.0;
        elevatorConfig.Slot1.kD = 0.0;
        //set slot 2 gains (climber)
        elevatorConfig.Slot2.kG = 0.0;
        elevatorConfig.Slot2.kV = 0.0;
        elevatorConfig.Slot2.kA = 0.0;

        elevatorConfig.Slot2.kP = 0.0;
        elevatorConfig.Slot2.kD = 0.0;
        

        // set Motion Magic Velocity settings
        elevatorConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                        Units.radiansToRotations(ElevatorConstants.profileConstraints.crusieVelocityRadPerSec()))
                .withMotionMagicAcceleration(ElevatorConstants.profileConstraints.accelerationRadPerSec2());

        leaderTalonFX.getConfigurator().apply(elevatorConfig);

        
        CANcoderConfiguration magConfig = new CANcoderConfiguration();

        magConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

        leaderCANcoder.getConfigurator().apply(magConfig);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.currentElevatorHeight = leaderTalonFX.getPosition().getValueAsDouble() * ElevatorConstants.unitsToRotations;//leaderCANcoder.getAbsolutePosition().getValueAsDouble();
        
    }

    @Override
    public void setHeight(double height) {
        //It is assumed that we would measure out the rotation of motors in degrees/radians/whatever in AdvantageScope for each novel position we want.
        //You don't have to do radians, i used it because it seems more easy to read compared to degrees/rotations, put rotations directly if its too much of a hassle.
        
        //convert height to rotations
        double convertedRot = height * ElevatorConstants.unitsToRotations;
         if (height > currentElevatorHeight){
            leaderTalonFX.setControl(new MotionMagicTorqueCurrentFOC(convertedRot).withSlot(0));
        } else {
            leaderTalonFX.setControl(new MotionMagicTorqueCurrentFOC(convertedRot).withSlot(1));
        }

        currentElevatorHeight = height;
    }

    @Override
    public void setBrakeMode(boolean leaderFlywheelBrake, boolean followerFlywheelBrake) {
        leaderTalonFX.setNeutralMode(leaderFlywheelBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerTalonFX.setNeutralMode(followerFlywheelBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    // @Override
    // public void setPosition(double position, double acceleration) {

    //     m_VelocityVoltage.Acceleration = acceleration;
    //     leaderTalonFX.setControl(m_VelocityVoltage.withVelocity(position));

    //     elevatorPosition = position;
    // }

    // @Override
    // public void setRealPosition(double position) {
    //     if (position > elevatorPosition){
    //         leaderTalonFX.setControl(new MotionMagicTorqueCurrentFOC(Units.degreesToRotations(position)).withSlot(0));
    //     } else {
    //         leaderTalonFX.setControl(new MotionMagicTorqueCurrentFOC(Units.degreesToRotations(position)).withSlot(1));
    //     }
    // }

    

}
