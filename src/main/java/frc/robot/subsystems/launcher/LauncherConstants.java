package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class LauncherConstants {

        public static Rotation2d minAngle = Rotation2d.fromDegrees(0.0);
        public static Rotation2d maxAngle = Rotation2d.fromDegrees(45.0);

        public static ProfileConstraints profileConstraints = new ProfileConstraints(4 * Math.PI, 10000);

        public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};

        public static Translation2d launcherOrigin =
        new Translation2d(-Units.inchesToMeters(12.0), Units.inchesToMeters(8.0));

}
