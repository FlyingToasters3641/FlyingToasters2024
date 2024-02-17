package frc.robot.controllers;

import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

public class ShotController {
    private PIDController headingController;
    SwerveDrivePoseEstimator drivePoseEstimator;

    NavigableMap<Double, Double> distanceAngles;

    public ShotController(SwerveDrivePoseEstimator mPoseEstimator) {
        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        drivePoseEstimator = mPoseEstimator;

        distanceAngles = new TreeMap<Double, Double>();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put(Units.inchesToMeters(36.0), 65.0);
        distanceAngles.put(Units.inchesToMeters(50.0), 62.0);
        distanceAngles.put(Units.inchesToMeters(60.0), 57.0);
        distanceAngles.put(Units.inchesToMeters(70.0), 53.0);
        distanceAngles.put(Units.inchesToMeters(84.0), 51.0);
        distanceAngles.put(Units.inchesToMeters(108.0), 45.0);
        distanceAngles.put(Units.inchesToMeters(117.0), 42.0);
        distanceAngles.put(Units.inchesToMeters(128.0), 38.0);
        distanceAngles.put(Units.inchesToMeters(138.0), 34.0);
        distanceAngles.put(Units.inchesToMeters(144.0), 32.5);
        distanceAngles.put(Units.inchesToMeters(168.0), 28.0);
        distanceAngles.put(Units.inchesToMeters(180.0), 25.0);
        distanceAngles.put(Units.inchesToMeters(204.0), 20.0);
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

    public double nearestSetpoint(NavigableMap<Double, Double> distMap, double distance) {
        double output = distMap.floorEntry(distance).getValue();
        Logger.recordOutput("ShotControl/LauncherAngle", output);
        Logger.recordOutput("ShotControl/Distance", Units.metersToInches(distance));
        
        return output;
    }
}
