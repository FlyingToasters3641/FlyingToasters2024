// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
        private static final double DEADBAND = 0.1;
        static PIDController lime = new PIDController(5.0, 0, 0);

        private DriveCommands() {
        }

        /**
         * Field relative drive command using two joysticks (controlling linear and
         * angular velocities).
         * 
         * @param drive Drivetrain Subsystem
         * @param xSupplier the left and right of the left joystick
         * @param ySupplier the up and down of the left joystick
         * @param omegaSupplier the right joystick for rotation
         * @param m_Limelight Limelight Subsystem
         * @return the command that drives the robot
         * 
         */
        public static Command joystickDrive(
                        DriveSubsystem drive,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier,
                        Limelight m_Limelight) {
                return Commands.run(() -> {
                        // Apply deadband
                        double linearMagnitude = MathUtil.applyDeadband(
                                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                        Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                        // Square values
                        linearMagnitude = linearMagnitude * linearMagnitude;
                        omega = Math.copySign(omega * omega * omega * omega, omega);

                        // Calcaulate new linear velocity
                        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                        .getTranslation();

                        // Convert to field relative speeds & send command
                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red;
                        // Auto aim steering takeover
                        if (drive.getAimController()) {
                                omega = drive.updateAimController(m_Limelight)
                                                / drive.getMaxAngularSpeedRadPerSec();
                        }
                        if (drive.getLobController()) {
                                omega = drive.updateLobController()
                                                / drive.getMaxAngularSpeedRadPerSec();
                        }

                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                        omega * drive.getMaxAngularSpeedRadPerSec(),
                                        isFlipped
                                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                        : drive.getRotation()));

                },
                                drive);
        }

        /**
         * 
         * Old WIP rotation autoaim for autonomous mode
         * 
         * 
         * @param drive Drivetrain subsystem
         * @param m_Limelight Limelight Subsystem
         * 
         */
        public static Command AutoAutoAim(
                        DriveSubsystem drive,
                        Limelight m_Limelight) {

                return Commands.run(
                                () -> {
                                        double omega = 0.0;
                                        // Convert to field relative speeds & send command

                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        // Auto aim steering takeover
                                        omega = lime.calculate(m_Limelight.getAngleOffset().getRadians(), 0); // will
                                                                                                              // get
                                                                                                              // multiplied
                                        // back later

                                        Logger.recordOutput("AutoAim/Omega", omega);
                                        drive.runVelocity(
                                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                        0.0,
                                                                        0.0,
                                                                        omega,
                                                                        isFlipped
                                                                                        ? drive.getRotation().plus(
                                                                                                        new Rotation2d(Math.PI))
                                                                                        : drive.getRotation()));

                                });
        }


    /** Attempts to stop the drive by putting the wheels in an x formation
     * 
     * @param drive Drivetrain Subsystme
     * @return the method in drive subsystem that stops the drive
     */
        public static Command DriveXStop(DriveSubsystem drive) {
                return Commands.run(() -> {
                        drive.stopWithX();
                });
        }

}
