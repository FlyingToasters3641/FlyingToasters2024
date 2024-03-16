package frc.robot.controllers;

import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;


import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class ShotController {
    private PIDController headingController;

    Limelight limelight;

    InterpolatingDoubleTreeMap distanceAngles;

    public ShotController() {


        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);


        distanceAngles = new InterpolatingDoubleTreeMap();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put((36.0), 45.0);
        distanceAngles.put((50.0), 40.0);
        distanceAngles.put((60.0), 49.0);
        distanceAngles.put((70.0), 35.0);
        distanceAngles.put((84.0), 28.0);
        distanceAngles.put((108.0), 21.0);
        distanceAngles.put((117.0), 19.0);
        distanceAngles.put((128.0), 17.0);
        distanceAngles.put((168.0), 14.0);
        distanceAngles.put((180.0), 11.0);
        distanceAngles.put((204.0), 9.5);
    }



    public double updateAngle() {
        double distance = limelight.getArea();
        double output = nearestSetpoint(distanceAngles, distance);
        Logger.recordOutput("AutoAim/DistanceToTarget", distance);
        
        return output;
    }

    public double nearestSetpoint(InterpolatingDoubleTreeMap distMap, double distance) {
        
        return distMap.get(distance);
    }
}
