package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class LauncherConstants {

        public static Rotation2d minAngle = Rotation2d.fromDegrees(0.0);
        public static Rotation2d maxAngle = Rotation2d.fromDegrees(45.0);

        public static ProfileConstraints profileConstraints = new ProfileConstraints(2 * Math.PI, (1 * Math.PI));

        public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};

        public static Translation2d launcherOrigin =
        new Translation2d(-Units.inchesToMeters(12.0), Units.inchesToMeters(8.0));

        public static double IDLE = 4.0;
        public static double FLYWHEEL_RPM_IDLE = 15;
        public static double FLYWHEEL_RPM_DEFAULT = 40;
        public static double FLYWHEEL_RPM_FAR = 23;
        public static double FLYWHEEL_RPM_INTAKE = -1000;
        public static double FLYWHEEL_RPM_AMP = 8;

}
