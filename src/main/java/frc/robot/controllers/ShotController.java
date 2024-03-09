package frc.robot.controllers;

import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

public class ShotController {
    private PIDController headingController;
    SwerveDrivePoseEstimator drivePoseEstimator;

    InterpolatingDoubleTreeMap distanceAngles;

    public ShotController(SwerveDrivePoseEstimator mPoseEstimator) {
        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        drivePoseEstimator = mPoseEstimator;

        distanceAngles = new InterpolatingDoubleTreeMap();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put(Units.inchesToMeters(36.0), 46.0);
        distanceAngles.put(Units.inchesToMeters(50.0), 42.0);
        distanceAngles.put(Units.inchesToMeters(60.0), 40.0);
        distanceAngles.put(Units.inchesToMeters(70.0), 34.0);
        distanceAngles.put(Units.inchesToMeters(84.0), 28.0);
        distanceAngles.put(Units.inchesToMeters(108.0), 23.0);
        distanceAngles.put(Units.inchesToMeters(117.0), 20.0);
        distanceAngles.put(Units.inchesToMeters(128.0), 17.0);
        distanceAngles.put(Units.inchesToMeters(168.0), 14.0);
        distanceAngles.put(Units.inchesToMeters(180.0), 11.0);
        distanceAngles.put(Units.inchesToMeters(204.0), 9.5);
    }

    public double updateDrive() {
        var aimingParameters = new RobotState().getAimingParameters(drivePoseEstimator);
        double output = headingController.calculate(
                drivePoseEstimator.getEstimatedPosition().getRotation().getRadians(),
                aimingParameters.driveHeading().getRadians());

        Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
        return output;
    }

    public double updateAngle() {
        double output = nearestSetpoint(distanceAngles, new RobotState().distanceToTarget(drivePoseEstimator));

        
        return output;
    }

    public double nearestSetpoint(InterpolatingDoubleTreeMap distMap, double distance) {
        
        return distMap.get(distance);
    }
}
