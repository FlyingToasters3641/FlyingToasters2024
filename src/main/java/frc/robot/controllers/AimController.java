package frc.robot.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class AimController {

    private PIDController headingController;
    private static final double Threshold = 1.0;
    SwerveDrivePoseEstimator drivePoseEstimator;
    
    public AimController(SwerveDrivePoseEstimator mPoseEstimator) {
        headingController = new PIDController(5.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Threshold);
        drivePoseEstimator = mPoseEstimator;
    }

    public double update(Limelight limelight) {
        double output = 0.0;
        if (limelight.getAngleOffset().getDegrees() == 0.0){
            var AimingParameters = new RobotState().getAimingParameters(drivePoseEstimator, GeomUtil.toTransform2d(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
                .toTranslation2d()));
            output = headingController.calculate(
                    drivePoseEstimator.getEstimatedPosition().getRotation().getRadians(),
                    AimingParameters.driveHeading().getRadians());
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
