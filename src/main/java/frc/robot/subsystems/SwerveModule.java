// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_steerMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_steerEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

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
   * @param krearleftdriveencoderports The channels of the drive encoder.
   * @param steerEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param steerEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int steerMotorChannel,
      int krearleftdriveencoderports,
      int krearleftsteerencoderports,
      boolean driveEncoderReversed,
      boolean steerEncoderReversed) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless); //TODO: BRUSHED OR BRUSHLESS?

    m_driveEncoder = new Encoder(krearleftdriveencoderports, krearleftdriveencoderports);

    m_steerEncoder = new Encoder(krearleftsteerencoderports, krearleftsteerencoderports);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_steerEncoder.setDistancePerPulse(ModuleConstants.kSteerEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    m_steerEncoder.setReverseDirection(steerEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_SteerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(m_steerEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_steerEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_steerEncoder.getDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_SteerPIDController.calculate(m_steerEncoder.getDistance(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_steerMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_steerEncoder.reset();
  }
}
