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


    InterpolatingDoubleTreeMap distanceAngles;
    double tx = 0.0;
    double distance;

    public ShotController() {


        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);


        distanceAngles = new InterpolatingDoubleTreeMap();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put((45.0), 45.0);
        distanceAngles.put((40.0), 40.0);
        distanceAngles.put((30.0), 49.0);
        distanceAngles.put((24.0), 35.0);
        distanceAngles.put((20.0), 28.0);
        distanceAngles.put((18.0), 21.0);
        distanceAngles.put((16.0), 19.0);
        distanceAngles.put((14.0), 17.0);
        distanceAngles.put((13.0), 13.0);
        distanceAngles.put((12.5), 11.0);
        distanceAngles.put((12.0), 10.0);
        distanceAngles.put((10.0), 9.0);
        distanceAngles.put((8.0), 8.0);
    }



    public double updateAngle(Limelight limelight) {
        if (limelight.gettY() == 0.0) {
        double distance = tx;
        } else {
        double distance = limelight.gettY();
        tx = distance;
        }
        double output = nearestSetpoint(distanceAngles, distance);
        Logger.recordOutput("AutoAim/DistanceToTarget", distance);
        
        return output;
    }

    public double nearestSetpoint(InterpolatingDoubleTreeMap distMap, double distance) {
        
        return distMap.get(distance);
    }
}
