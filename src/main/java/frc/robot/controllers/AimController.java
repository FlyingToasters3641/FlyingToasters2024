package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.RobotState;

public class AimController {

    private PIDController headingController;
    SwerveDrivePoseEstimator drivePoseEstimator;

    public AimController(SwerveDrivePoseEstimator mPoseEstimator) {
        headingController = new PIDController(6.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        drivePoseEstimator = mPoseEstimator;
    }

    public double update() {
        var aimingParameters = new RobotState().getAimingParameters(drivePoseEstimator);
        double output = headingController.calculate(
                drivePoseEstimator.getEstimatedPosition().getRotation().getRadians(),
                aimingParameters.driveHeading().getRadians());

        Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
        return output;
    }

}
