package frc.robot.controllers;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TrapController {
    private PIDController headingController;
    private static final double Threshold = 1.0;
    
    public TrapController() {
        headingController = new PIDController(5.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Threshold);
        
    }

    public double update(PhotonCamera vision) {
        double output = 0.0;
        List<PhotonTrackedTarget> targets = vision.getLatestResult().getTargets();
        Logger.recordOutput("AutoAim/Camera", vision.getLatestResult().hasTargets());
        PhotonTrackedTarget target = null;
        for (int i = 0; i < targets.size(); i++){
            if (DriverStation.getAlliance().get() == Alliance.Blue){
                if (targets.get(i).getFiducialId() == 14){
                    target = targets.get(i);
                    break;
                } else if (targets.get(i).getFiducialId() == 15){
                    target = targets.get(i);
                    break;
                } else if (targets.get(i).getFiducialId() == 16){
                    target = targets.get(i);
                    break;
                }
            } else if (DriverStation.getAlliance().get() == Alliance.Red){
                if (targets.get(i).getFiducialId() == 11){
                    target = targets.get(i);
                    break;
                } else if (targets.get(i).getFiducialId() == 12){
                    target = targets.get(i);
                    break;
                } else if (targets.get(i).getFiducialId() == 13){
                    target = targets.get(i);
                    break;
                }
            }
            
        }
        if (target != null){
            output = headingController.calculate(
                Units.degreesToRadians(target.getYaw()),  0);
        }     
        
        return output;
    }
}
