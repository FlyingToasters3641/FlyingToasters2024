// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {

    //TODO: ENCODER PORTS ARE MESSED UP FIX THEM LATER

    //Front Left 

    public static final int kFrontLeftDriveMotorId = 30;
    public static final int kFrontLeftSteerMotorId = 32;
    public static final int kFrontLeftDriveEncoderPorts = 0; 
    public static final int kFrontLeftSteerEncoderPorts = 0;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftSteerEncoderReversed = false;
    

    //Front Right
    public static final int kFrontRightDriveMotorId = 36;
    public static final int kFrontRightSteerMotorId = 33;
    public static final int kFrontRightDriveEncoderPorts = 0;
    public static final int kFrontRightSteerEncoderPorts = 0;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightSteerEncoderReversed = false;
    
    //Rear Left
    public static final int kRearLeftDriveMotorId = 42;
    public static final int kRearLeftSteerMotorId = 34;
    public static final int kRearLeftDriveEncoderPorts = 0;
    public static final int kRearLeftSteerEncoderPorts = 0;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftSteerEncoderReversed = true;


    //Rear Right
    public static final int kRearRightDriveMotorId = 37;
    public static final int kRearRightSteerMotorId = 35;
    public static final int kRearRightDriveEncoderPorts = 0;
    public static final int kRearRightSteerEncoderPorts = 0;
    public static final boolean kRearRightDriveEncoderReversed = true;
    public static final boolean kRearRightSteerEncoderReversed = true;

    
    //Canbus Name and PigeonIDs
    public static final String CANbusName = "Lucas";
    public static final int Pigeon2ID = 0;
   
    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDefaultPeriod = 0.02;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kSteerEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleSteerController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
