package frc.robot.controllers;

import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

public class ShotController {
    private PIDController headingController;

    NavigableMap<Double, Double> distanceAngles;

    public ShotController() {
        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        distanceAngles = new TreeMap<Double, Double>();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put(Units.inchesToMeters(36.0), 68.0);
        distanceAngles.put(Units.inchesToMeters(60.0), 55.0);
        distanceAngles.put(Units.inchesToMeters(84.0), 45.0);
        distanceAngles.put(Units.inchesToMeters(108.0), 35.0);
        distanceAngles.put(Units.inchesToMeters(144.0), 30.0);
        distanceAngles.put(Units.inchesToMeters(168.0), 25.0);
        distanceAngles.put(Units.inchesToMeters(204.0), 20.0);
    }

    public double updateDrive() {
        var aimingParameters = RobotState.getInstance().getAimingParameters();
        double output = headingController.calculate(
                RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                aimingParameters.driveHeading().getRadians());

        Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
        return output;
    }

    public double updateAngle() {
        double output = nearestSetpoint(distanceAngles, RobotState.getInstance().getAimingParameters().distanceToTarget());

        Logger.recordOutput("AutoAim/LauncherAngle", output);
        return output;
    }

    public double nearestSetpoint(NavigableMap<Double, Double> distMap, double distance) {
        return distMap.floorKey(distance);
    }
}
