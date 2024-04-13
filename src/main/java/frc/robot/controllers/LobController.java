package frc.robot.controllers;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;

public class LobController {

    private PIDController headingController;
    SwerveDrivePoseEstimator drivePoseEstimator;
    private static final double Threshold = 1.0;
    
    public LobController(SwerveDrivePoseEstimator mPoseEstimator) {
        headingController = new PIDController(3.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Threshold);
        drivePoseEstimator = mPoseEstimator;
    }

    public double update() {
        var AimingParameters = new RobotState().getAimingParameters(drivePoseEstimator);
        double output = headingController.calculate(
                drivePoseEstimator.getEstimatedPosition().getRotation().getRadians(),
                AimingParameters.driveHeading().getRadians());

            Logger.recordOutput("AutoLobAim/HeadingError", headingController.getPositionError());
            return output;
    }

}
