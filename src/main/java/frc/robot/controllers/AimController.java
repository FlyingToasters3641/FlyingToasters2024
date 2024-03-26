package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class AimController {

    private PIDController headingController;
    private static final double Threshold = 2.0;
    
    public AimController() {
        headingController = new PIDController(5.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        
        
    }

    public double update(Limelight limelight) {
        double output = headingController.calculate(
                limelight.getAngleOffset().getRadians(),
                0);
                
        
        Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
        Logger.recordOutput("AutoAim/HeadingDegrees", Units.radiansToDegrees(output));
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
