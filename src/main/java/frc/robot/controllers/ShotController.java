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
    double tx;
    double distance;


    InterpolatingDoubleTreeMap distanceAngles;

    InterpolatingDoubleTreeMap autoLobAngles;

    InterpolatingDoubleTreeMap autoLobRollers;

    public ShotController() {


        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);


        distanceAngles = new InterpolatingDoubleTreeMap();
        autoLobAngles = new InterpolatingDoubleTreeMap();
        autoLobRollers = new InterpolatingDoubleTreeMap();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put((45.0), 45.0);
        distanceAngles.put((40.0), 40.0);
        distanceAngles.put((30.0), 40.0);
        distanceAngles.put((24.0), 36.0);
        distanceAngles.put((20.0), 30.0);
        distanceAngles.put((18.0), 21.0);
        distanceAngles.put((16.0), 20.0);
        distanceAngles.put((14.0), 19.0);
        distanceAngles.put((13.0), 18.0);
        distanceAngles.put((12.5), 17.0);
        distanceAngles.put((12.0), 15.0);
        distanceAngles.put((11.0), 14.0);
        distanceAngles.put((10.0), 12.0);
        distanceAngles.put((9.0), 10.0);
        distanceAngles.put((8.0), 9.5);
        distanceAngles.put((7.0), 8.0);

        //Auto Lob Angles - Needs Calibration + More Entries
        autoLobAngles.put((9.0), 40.0);
        autoLobAngles.put((7.0), 30.0);
        autoLobAngles.put((5.0), 27.5);

        //Auto Lob Roller - Needs Calibration + More Entries
        autoLobRollers.put((9.0), 23.0);
        autoLobRollers.put((7.0), 24.0);
        autoLobRollers.put((5.0), 25.0);


    }



    public double updateAngle(Limelight limelight) {
        if (limelight.getY() == 0.0) {
            distance = tx;
        } else {
        distance = limelight.getY();
        tx = distance;
        }
        double output = nearestSetpoint(distanceAngles, distance);
        Logger.recordOutput("AutoAim/DistanceToTarget", distance);
        
        return output;
    }

    public double updateLobbedAngle(Limelight limelight){
        double distance = limelight.getY();
        double output = nearestSetpoint(autoLobAngles, distance);
        Logger.recordOutput("AutoLobAim/DistanceToTarget", distance);

        return output;
    }

    public double updateLobbedRollers(Limelight limelight) {
        double distance = limelight.getY();
        double output = nearestSetpoint(autoLobRollers, distance);

        return output;

    }


    public double nearestSetpoint(InterpolatingDoubleTreeMap distMap, double distance) {
        
        return distMap.get(distance);
    }
}
