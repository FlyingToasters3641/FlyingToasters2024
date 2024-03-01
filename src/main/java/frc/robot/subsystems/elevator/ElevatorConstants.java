package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {


    //NOT TESTED YET!!!
    public static ProfileConstraints profileConstraints = new ProfileConstraints(0.0,0.0);

    public record ProfileConstraints(double crusieVelocityRadPerSec, double accelerationRadPerSec2) {};

    //For calculating height
    public static final double unitsToRotations = 0.0;


    //Use this space to add CONSTANT values for set positions of elevator height when needed


}