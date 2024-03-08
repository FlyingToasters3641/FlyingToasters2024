package frc.robot.subsystems.elevator;

public class ElevatorConstants {

    public static ProfileConstraints profileConstraints = new ProfileConstraints(4 * Math.PI, (8 * Math.PI));

	public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};
    
}
