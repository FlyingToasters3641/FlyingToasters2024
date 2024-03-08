// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class SwerveModuleComp implements ModuleIO {
  // Can bus 
  public static final String CANbusName = "Lucas";

  // Gear ratios for Swerve XS, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = 4.40;
  private static final double TURN_GEAR_RATIO = 41.25;

  // Motor Objects
  private final TalonFX driveTalon;
  private final CANSparkMax turnSparkMax;

  // Encoder Objects
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;

  // Position/Velocity
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> turnAbsolutePosition;
  
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  

  public SwerveModuleComp(int index) {
    switch (index) {
      case 0: // Front Left
        driveTalon = new TalonFX(13, CANbusName);
        turnSparkMax = new CANSparkMax(20, MotorType.kBrushless);
        cancoder = new CANcoder(17, CANbusName);
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.252441)); // MUST BE CALIBRATED
        break;
      case 1: // Front Right
        driveTalon = new TalonFX(10, CANbusName);
        turnSparkMax = new CANSparkMax(21, MotorType.kBrushless);
        cancoder = new CANcoder(15, CANbusName);
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(-0.109619)); // MUST BE CALIBRATED
        break;
      case 2: // Rear Left
        driveTalon = new TalonFX(11, CANbusName);
        turnSparkMax = new CANSparkMax(22, MotorType.kBrushless);
        cancoder = new CANcoder(16, CANbusName);
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.189453)); // MUST BE CALIBRATED
        break;
      case 3: // Rear Right
        driveTalon = new TalonFX(12, CANbusName);
        turnSparkMax = new CANSparkMax(19, MotorType.kBrushless);
        cancoder = new CANcoder(18, CANbusName);
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(-0.047119)); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Initialize TalonFX Config
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    driveConfig.Slot0.kS = 0.317;
    driveConfig.Slot0.kV = 0.68;
    driveConfig.Slot0.kP = 0.1;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // Initialize SparkMax Config
    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);

    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.enableVoltageCompensation(12.0);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // May need to adjust, this value can cause  CAN timeout

    turnSparkMax.setCANTimeout(0);

    // Initialize CANCoder Config
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    turnAbsolutePosition = cancoder.getAbsolutePosition();

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    

    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(() -> driveTalon.getPosition().getValue());
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
      Module.ODOMETRY_FREQUENCY, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition);
    driveTalon.optimizeBusUtilization();

    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] { driveCurrent.getValueAsDouble() };

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
        .minus(absoluteEncoderOffset);

    inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
        / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
      turnPositionQueue.stream()
          .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
          .toArray(Rotation2d[]::new);
        
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts, true, false, false, false));
  }

  @Override
  public void setDriveVelocity(double velocity){
    driveTalon.setControl(new VelocityVoltage(velocity));
  }


  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
