package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class AimController {

    private PIDController headingController;

    Limelight limelightHelpers;
    

    public AimController() {
        headingController = new PIDController(6.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        
        
    }

    public double update() {
        double output = headingController.calculate(
                0,
                limelightHelpers.getAngleOffset().getRadians());
                
        
        Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
        Logger.recordOutput("AutoAim/HeadingDegrees", Units.radiansToDegrees(output));
        return output;
    }

}
