package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class ShotController {
    private PIDController headingController;
    double tx;
    double distance;


    InterpolatingDoubleTreeMap distanceAngles;
    InterpolatingDoubleTreeMap farAngles;
    LinearFilter filter = LinearFilter.movingAverage(10);

    InterpolatingDoubleTreeMap autoLobAngles;
    InterpolatingDoubleTreeMap autoLobRollers;
    InterpolatingDoubleTreeMap distanceRollers;
    InterpolatingDoubleTreeMap distanceDemoRollers;

    SwerveDrivePoseEstimator drivePoseEstimator;

    public ShotController(SwerveDrivePoseEstimator mPoseEstimator) {


        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        drivePoseEstimator = mPoseEstimator;

        distanceAngles = new InterpolatingDoubleTreeMap();
        autoLobAngles = new InterpolatingDoubleTreeMap();
        autoLobRollers = new InterpolatingDoubleTreeMap();
        distanceDemoRollers = new InterpolatingDoubleTreeMap();
        
        //Angle distance pairs - Needs Calibration
        //Input: Robot Angle to April Tag *** Output: Angle of the End Effector to Shoot
        distanceAngles.put((45.0), 46.0);
        distanceAngles.put((40.0), 42.0);
        distanceAngles.put((30.0), 40.0);
        distanceAngles.put((24.0), 36.0);
        distanceAngles.put((20.0), 30.0);
        distanceAngles.put((18.0), 25.0);
        distanceAngles.put((16.0), 21.0);
        distanceAngles.put((14.0), 17.5);
        distanceAngles.put((13.0), 14.5);
        distanceAngles.put((12.5), 13.0);
        distanceAngles.put((12.0), 12.75);
        distanceAngles.put((11.0), 10.5);
        distanceAngles.put((10.0), 8.25);
        distanceAngles.put((9.0), 5.5);
        distanceAngles.put((8.0), 5.0);

        //For Photon Camera
        farAngles = new InterpolatingDoubleTreeMap();
        farAngles.put((17.0), 7.8);
        farAngles.put((16.5), 7.4);
        farAngles.put((16.0), 7.0);
        farAngles.put((15.0), 6.5);
        farAngles.put((14.5), 6.0);
        farAngles.put((14.0), 5.5);
        //farAngles.put((15.5), 5.0);
        //farAngles.put((15.0), 4.5);

        autoLobAngles.put((9.0), 30.0);
        autoLobAngles.put((7.0), 30.0);
        autoLobAngles.put((5.0), 30.0);

        //AutoLobRollers
        autoLobRollers.put((10.0), 25.0);
        autoLobRollers.put((9.0), 24.0);
        autoLobRollers.put((8.0), 23.0);
        
        //DistanceRollers
        distanceRollers = new InterpolatingDoubleTreeMap();
        distanceRollers.put((45.0), 25.0);
        distanceRollers.put((30.0), 25.0);
        distanceRollers.put((23.0),  30.0);
        distanceRollers.put((15.0), 32.0);
        distanceRollers.put((12.0), 35.0);
        distanceRollers.put((11.0), 38.0);
        distanceRollers.put((10.0), 40.0);
        distanceRollers.put((8.0), 45.0);

    }

    public double updateAngle(Limelight limelight) {
        double output = 1.0;
        double distance = 0.0;
        if (limelight.getY() == 0.0) {
            distance = tx;
        } else {
        distance = limelight.getY();
        tx = distance;
        }
        output = nearestSetpoint(distanceAngles, distance);
        Logger.recordOutput("AutoAim/CloseTarget", distance);
        Logger.recordOutput("AutoAim/CloseOutput", output);
        SmartDashboard.putNumber("CloseShotDistance", distance);
        SmartDashboard.putNumber("CloseShotOutput", distance);
        
        
        return output;
    }

    public double updateRollers(Limelight limelight, boolean demo) {
        double output = 1.0;
        double distance = 0.0;
        if (limelight.getY() == 0.0) {
            distance = tx;
        } else {
        distance = limelight.getY();
        tx = distance;
        }   
        if (demo) {
            output = (20);
        } else {
        output = nearestSetpoint(distanceRollers, distance);
        Logger.recordOutput("AutoAim/Rollers", output);
        }
        return output;
    }

    public double updateLobbedAngle(Limelight limelight){
        double distance = new RobotState().distanceToTarget(drivePoseEstimator);
        double output = nearestSetpoint(autoLobAngles, distance);
        Logger.recordOutput("AutoLobAim/DistanceToTarget", distance);

        return output;
    }

    public double updateLobbedRollers(Limelight limelight) {
        double distance = new RobotState().distanceToTarget(drivePoseEstimator);
        double output = nearestSetpoint(autoLobRollers, distance);

        return output;

    }

    public double nearestSetpoint(InterpolatingDoubleTreeMap distMap, double distance) {
        
        return distMap.get(distance);
    }
}
