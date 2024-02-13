package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
    public record ModuleLimits(
            double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {
    }

    public record DriveConfig(
            double wheelRadius,
            double trackwidthX,
            double trackwidthY,
            double maxLinearVelocity,
            double maxLinearAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration) {
        public double driveBaseRadius() {
            return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
        }
    }

    public static DriveConfig driveConfig = new DriveConfig(
            Units.inchesToMeters(1.5),
            Units.inchesToMeters(20.5),
            Units.inchesToMeters(20.5),
            Units.feetToMeters(17.5),
            Units.feetToMeters(30.02),
            8.86,
            43.97);

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(driveConfig.trackwidthX() / 2.0, driveConfig.trackwidthY() / 2.0),
            new Translation2d(driveConfig.trackwidthX() / 2.0, -driveConfig.trackwidthY() / 2.0),
            new Translation2d(-driveConfig.trackwidthX() / 2.0, driveConfig.trackwidthY() / 2.0),
            new Translation2d(-driveConfig.trackwidthX() / 2.0, -driveConfig.trackwidthY() / 2.0)
    };
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

    public static final Matrix<N3, N1> odometryStateStdDevs = switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
    };

    public static ModuleLimits moduleLimits = new ModuleLimits(
            driveConfig.maxLinearVelocity(),
            driveConfig.maxLinearVelocity() * 5,
            Units.degreesToRadians(1080.0));
}
