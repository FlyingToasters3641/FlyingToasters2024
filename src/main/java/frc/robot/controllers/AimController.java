package frc.robot.controllers;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class AimController {

    private PIDController headingController;
    private static final double Threshold = 1.0;
    
    public AimController() {
        headingController = new PIDController(5.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Threshold);
        
    }

    public double update(Limelight limelight, PhotonCamera vision) {
        double output = 0.0;
        List<PhotonTrackedTarget> targets = vision.getLatestResult().getTargets();
        Logger.recordOutput("AutoAim/Camera", vision.getLatestResult().hasTargets());
        PhotonTrackedTarget target = null;
        for (int i = 0; i < targets.size(); i++){
            if (targets.get(i).getFiducialId() == 7){
                target = targets.get(i);
                Logger.recordOutput("AutoAim/Target", targets.get(i).getYaw());
                break;
            }
        }
        if (target != null){
            output = headingController.calculate(
                Units.degreesToRadians(target.getYaw()), 0);
            Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
            Logger.recordOutput("AutoAim/HeadingDegrees", Units.radiansToDegrees(output));
        }else{
        output = headingController.calculate(
                limelight.getAngleOffset().getRadians(),
                0);
        }        
        
        
        return output;
    }

    public double updateLobBlue(Limelight limelight) {
        double output = headingController.calculate(
            limelight.getAngleOffset().getRadians() - Units.degreesToRadians(15),
                        0);

        return output;
    }
    
    public double updateLobRed(Limelight limelight) {
        double output = headingController.calculate(
            limelight.getAngleOffset().getRadians() - Units.degreesToRadians(15),
                        0);

        return output;
    }
    



    public boolean threshold(Limelight limelight) {
        double output = limelight.getX();
        if ((Math.abs(output) <= Threshold)){
            return true;
        } else {
            return false;
        }
    }

}
