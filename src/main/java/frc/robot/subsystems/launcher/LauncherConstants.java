package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;

public class LauncherConstants {

        public static final double reduction = (1.0/2.0);

        public static Rotation2d minAngle = Rotation2d.fromDegrees(0.0);
        public static Rotation2d maxAngle = Rotation2d.fromDegrees(45.0);

        public static ProfileConstraints profileConstraints = new ProfileConstraints(2 * Math.PI, 10);

        public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};

        public static class FlywheelConstants {
                
                //TODO: UPDATE THIS CRUD
                public static Gains gains = new Gains(0, 0, 0, 0.09078, 0.00103, 0);

                public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {};                        
        }

}
