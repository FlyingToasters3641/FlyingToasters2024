package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

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
        var AimingParameters = new RobotState().getAimingParameters(drivePoseEstimator, GeomUtil.toTransform2d(AllianceFlipUtil.apply(FieldConstants.Speaker.LobSpot)
                .toTranslation2d()));
        double output = headingController.calculate(
                drivePoseEstimator.getEstimatedPosition().getRotation().getRadians(),
                AimingParameters.driveHeading().getRadians());

            Logger.recordOutput("AutoLobAim/HeadingError", headingController.getPositionError());
            return output;
    }

}
