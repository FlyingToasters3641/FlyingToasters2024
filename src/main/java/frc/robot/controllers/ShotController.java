package frc.robot.controllers;

import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class ShotController {
    private PIDController headingController;
    double tx;
    double distance;


    InterpolatingDoubleTreeMap distanceAngles;
    InterpolatingDoubleTreeMap farAngles;
    LinearFilter filter = LinearFilter.movingAverage(5);

    InterpolatingDoubleTreeMap autoLobAngles;

    InterpolatingDoubleTreeMap autoLobRollers;

    public ShotController() {


        headingController = new PIDController(1.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);


        distanceAngles = new InterpolatingDoubleTreeMap();
        autoLobAngles = new InterpolatingDoubleTreeMap();
        autoLobRollers = new InterpolatingDoubleTreeMap();
        
        //Angle distance pairs - Needs Calibration
        distanceAngles.put((45.0), 46.0);
        distanceAngles.put((40.0), 42.0);
        distanceAngles.put((30.0), 40.0);
        distanceAngles.put((24.0), 36.0);
        distanceAngles.put((20.0), 30.0);
        distanceAngles.put((18.0), 25.0);
        distanceAngles.put((16.0), 22.0);
        distanceAngles.put((14.0), 19.0);
        distanceAngles.put((13.0), 18.0);
        distanceAngles.put((12.5), 16.5);
        distanceAngles.put((12.0), 15.0);
        distanceAngles.put((11.0), 11.0);
        distanceAngles.put((10.0), 9.0);
        distanceAngles.put((8.0), 7.0);

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

        autoLobAngles.put((9.0), 40.0);
        autoLobAngles.put((7.0), 30.0);
        autoLobAngles.put((5.0), 27.5);
        //AutoLobRollers

        autoLobRollers.put((9.0), 23.0);
        autoLobRollers.put((7.0), 24.0);
        autoLobRollers.put((5.0), 25.0);
        
    }



    public double updateAngle(Limelight limelight, PhotonCamera m_vision) {
        double output = 0.0;
        double distance = 0.0;
        List<PhotonTrackedTarget> targets = m_vision.getLatestResult().getTargets();
        PhotonTrackedTarget target = null;
        for (int i = 0; i < targets.size(); i++){
            if (targets.get(i).getFiducialId() == 7  && DriverStation.getAlliance().get() == Alliance.Blue){
                target = targets.get(i);
                break;
            } else if (targets.get(i).getFiducialId() == 4 && DriverStation.getAlliance().get() == Alliance.Red){
                target = targets.get(i);
                break;
            }
        }
        if (target != null){
            distance = filter.calculate(target.getPitch());
            output = nearestSetpoint(farAngles, distance);
            Logger.recordOutput("AutoAim/FarTarget", distance);
            SmartDashboard.putNumber("FarShotDistance", distance);
            SmartDashboard.putNumber("FarShotOutput", output);
        }else{
            if (limelight.getY() == 0.0) {
                distance = tx;
            } else {
            distance = limelight.getY();
            tx = distance;
            }
            output = nearestSetpoint(distanceAngles, distance);
            Logger.recordOutput("AutoAim/CloseTarget", distance);
            SmartDashboard.putNumber("CloseShotDistance", distance);
            SmartDashboard.putNumber("CloseShotOutput", distance);
        }
        
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
