// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {

  //Motor Objects
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_steerMotor;

  //Encoder Objects
  private final CANcoder m_steerEncoder;

  //Position/Velocity 
  private StatusSignal<Double> m_drivePosition;
  private StatusSignal<Double> m_driveVelocity;

  private double m_speedAt12VoltsMps;

  //Offsets for the CANcoder
  private double m_steerOffset;

  private BaseStatusSignal[] m_signals;
  private double motorEncoderPositionCoefficient = 0;
  private double motorVelocityCoefficient = 0;
  private double m_driveRotationsPerMeter = 0;
  private double m_couplingRatioDriveRotorToCANcoder;
  
  /* Drive Motor Controls */
  private final VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0);
  private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);
  private final VelocityTorqueCurrentFOC m_velocityTorqueSetter = new VelocityTorqueCurrentFOC(0);
  private SwerveModulePosition m_internalState = new SwerveModulePosition();
  private ClosedLoopOutputType m_driveClosedLoopOutput;

  /* Steer Motor Controls */
  private  ProfiledPIDController m_SteerPIDController = new ProfiledPIDController(
    ModuleConstants.kPModuleSteerController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel           The channel of the drive motor.
   * @param steerMotorChannel           The channel of the turning motor.
   * @param kfrontleftdriveencoderports The channels of the drive encoder.
   * @param steerEncoderChannels        The channels of the turning encoder.
   */

  public SwerveModule(
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannels,
      double steerEncoderOffset) {
    
    m_driveMotor = new TalonFX(driveMotorChannel, DriveConstants.CANbusName);
    m_steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless); 

    m_steerEncoder = new CANcoder(steerMotorChannel, DriveConstants.CANbusName);

    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

    talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonConfigs.Slot0 = DriveConstants.DriveMotorGains;
    talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.SlipCurrent;
    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.SlipCurrent;
    talonConfigs.CurrentLimits.StatorCurrentLimit = DriveConstants.SlipCurrent;
    talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    talonConfigs.MotorOutput.Inverted = DriveConstants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    StatusCode response = m_driveMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
        System.out
                .println("Talon ID " + DriveConstants.DriveMotorId + " failed config with error " + response.toString());
    }

    m_steerOffset = steerEncoderOffset;

    m_drivePosition = m_driveMotor.getPosition();
    m_driveVelocity = m_driveMotor.getVelocity();

    m_signals = new BaseStatusSignal[2];
    m_signals[0] = m_drivePosition;
    m_signals[1] = m_driveVelocity;

    m_velocityTorqueSetter.UpdateFreqHz = 0;
    m_voltageOpenLoopSetter.UpdateFreqHz = 0;

    motorEncoderPositionCoefficient = 2.0 * Math.PI / DriveConstants.kDriveGearRatio;
    motorVelocityCoefficient = Math.PI * DriveConstants.kDriveGearRatio * DriveConstants.kWheelRadiusInches * 10.0;
    /* Calculate the ratio of drive motor rotation to meter on ground */
    double rotationsPerWheelRotation = ModuleConstants.DriveMotorGearRatio;
    double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(ModuleConstants.WheelRadius);
    m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;

    m_speedAt12VoltsMps = DriveConstants.SpeedAt12VoltsMps;
    m_driveClosedLoopOutput = DriveConstants.DriveMotorClosedLoopOutput;

    m_SteerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModulePosition getPositionRefresh(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      m_drivePosition.refresh();
      m_driveVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
    double angle_rot = m_steerMotor.getEncoder().getPosition();

    /* And push them into a SwerveModuleState object to return */
    m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
    /* Angle is already in terms of steer rotations */
    m_internalState.angle = Rotation2d.fromRotations(angle_rot);

    return m_internalState;
  }

  // public void apply(SwerveModuleState state) {
  // var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

  // double angleToSetDeg = optimized.angle.getRotations();
  // m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
  // double velocityToSet = optimized.speedMetersPerSecond *
  // m_driveRotationsPerMeter;
  // m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
  // }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, DriveRequestType driveRequestType) {
    var optimized = SwerveModuleState.optimize(desiredState, m_internalState.angle);
    var encoderRotation = getSteerAngle();
    double angleToSetDeg = optimized.angle.getRotations();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;

    /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
    /* To reduce the "skew" that occurs when changing direction */
    double steerMotorError = angleToSetDeg - m_steerMotor.getEncoder().getPosition();
    /* If error is close to 0 rotations, we're already there, so apply full power */
    /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
    double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
    /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
    if (cosineScalar < 0.0) {
        cosineScalar = 0.0;
    }
    velocityToSet *= cosineScalar;

    /* Back out the expected shimmy the drive motor will see */
    /* Find the angular rate to determine what to back out */
    double azimuthTurnRps = m_steerMotor.getEncoder().getVelocity();
    /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
    double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
    velocityToSet -= driveRateBackOut;


    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_SteerPIDController.calculate(m_steerEncoder.getPositionSinceBoot().getValue(),
        state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    switch (driveRequestType) {
      case OpenLoopVoltage:
          /* Open loop ignores the driveRotationsPerMeter since it only cares about the open loop at the mechanism */
          /* But we do care about the backout due to coupling, so we keep it in */
          velocityToSet /= m_driveRotationsPerMeter;
          m_driveMotor.setControl(m_voltageOpenLoopSetter.withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0));
          break;

      case Velocity:
          switch (m_driveClosedLoopOutput) {
              case Voltage:
                  m_driveMotor.setControl(m_velocityVoltageSetter.withVelocity(velocityToSet));
                  break;

              case TorqueCurrentFOC:
                  m_driveMotor.setControl(m_velocityTorqueSetter.withVelocity(velocityToSet));
                  break;
          }
          break;
    }
    m_steerMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public double getDriveVelocity() {
    return m_driveMotor.getVelocity().getValue() * motorVelocityCoefficient;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
  }

  public Rotation2d getSteerAngle() {
    double motorAngleRadians = m_steerMotor.getEncoder().getPosition() * motorEncoderPositionCoefficient;
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < m_steerOffset) {
      motorAngleRadians += 2.0 * Math.PI;
    }

    return new Rotation2d(motorAngleRadians);
  }

}
