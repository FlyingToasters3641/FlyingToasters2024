// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;


  private final CANcoder m_steerEncoder;

  private StatusSignal<Double> m_drivePosition;
  private StatusSignal<Double> m_driveVelocity;
  private StatusSignal<Double> m_steerPosition;
  private StatusSignal<Double> m_steerVelocity;

  private double m_steerOffset;


  private BaseStatusSignal[] m_signals;
private double motorEncoderPositionCoefficient = 0;
private double motorVelocityCoefficient = 0;
  private PositionVoltage m_angleSetter = new PositionVoltage(0);
   private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);
  private double m_driveRotationsPerMeter = 0;
  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

      private SwerveModulePosition m_internalState = new SwerveModulePosition();

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_SteerPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleSteerController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param steerMotorChannel The channel of the turning motor.
   * @param kfrontleftdriveencoderports The channels of the drive encoder.
   * @param steerEncoderChannels The channels of the turning encoder.
   */

  public SwerveModule(
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannels,
      double steerEncoderOffset
      ) {
    m_driveMotor = new TalonFX(driveMotorChannel, DriveConstants.CANbusName);
    m_steerMotor = new TalonFX(steerMotorChannel, DriveConstants.CANbusName);//new CANSparkMax(steerMotorChannel, MotorType.kBrushless); //TODO: BRUSHED OR BRUSHLESS?



    m_steerEncoder = new CANcoder(steerMotorChannel, DriveConstants.CANbusName);

    m_steerOffset = steerEncoderOffset;

    m_drivePosition = m_driveMotor.getPosition();
    m_driveVelocity = m_driveMotor.getVelocity();
    m_steerPosition = m_steerEncoder.getPosition();
    m_steerVelocity = m_steerEncoder.getVelocity();
    

    m_signals = new BaseStatusSignal[4];
    m_signals[0] = m_drivePosition;
    m_signals[1] = m_driveVelocity;
    m_signals[2] = m_steerPosition;
    m_signals[3] = m_steerVelocity;
SwerveModuleConstants m_SwerveModuleConstants = new SwerveModuleConstants();

motorEncoderPositionCoefficient = 2.0 * Math.PI / DriveConstants.kDriveGearRatio;
motorVelocityCoefficient = Math.PI * DriveConstants.kDriveGearRatio * DriveConstants.kWheelRadiusInches * 10.0;
      /* Calculate the ratio of drive motor rotation to meter on ground */
    double rotationsPerWheelRotation = m_SwerveModuleConstants.DriveMotorGearRatio;
    double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(m_SwerveModuleConstants.WheelRadius);
    m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;

    m_SteerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  

  public SwerveModulePosition getPositionRefresh(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      m_drivePosition.refresh();
      m_driveVelocity.refresh();
      m_steerPosition.refresh();
      m_steerVelocity.refresh();
    }

    

    /* Now latency-compensate our signals */
    double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
    double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

    /* And push them into a SwerveModuleState object to return */
    m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
    /* Angle is already in terms of steer rotations */
    m_internalState.angle = Rotation2d.fromRotations(angle_rot);

    return m_internalState;
  }

// public void apply(SwerveModuleState state) {
//     var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

//     double angleToSetDeg = optimized.angle.getRotations();
//     m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
//     double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
//     m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
//   }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = getSteerAngle();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getVelocity().getValue(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_SteerPIDController.calculate(m_steerEncoder.getPositionSinceBoot().getValue(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
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
    double motorAngleRadians = m_steerMotor.getPosition().getValue() * motorEncoderPositionCoefficient;
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < m_steerOffset) {
      motorAngleRadians += 2.0 * Math.PI;
    }

    return new Rotation2d(motorAngleRadians);
  }

//   public SwerveModulePosition getPosition() {
//     return new SwerveModulePosition(
//       m_driveMotor.getPosition, getSteerAngle());
//   }
 }
