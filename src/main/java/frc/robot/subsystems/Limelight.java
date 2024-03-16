package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        

        // read values periodically
        
        double y = ty.getDouble(0.0);

        public double getArea(){
            return table.getEntry("ta").getDouble(0.0);
        }

        public Rotation2d getAngleOffset(){
            double x = tx.getDouble(0.0);
            return new Rotation2d(Units.degreesToRadians(x));
        }

}
