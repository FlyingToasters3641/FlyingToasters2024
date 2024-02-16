package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class AimController {
    public record AimingParameters(Rotation2d driveHeading, Rotation2d launcherAngle, double driveFeedVelocity) {
    };

    private final double LOOK_AHEAD = 0.0; //Tune to compensate for robot position latency 
    private final double HEIGHT_COMP = 0.0; //Tune to compensate for robot shot height

    private PIDController headingController;
    private Twist2d robotVelocity = new Twist2d();

    private AimingParameters lastParamters = null;

    private SwerveDrivePoseEstimator drivePoseEstimator;

    public AimController(SwerveDrivePoseEstimator mPoseEstimator) {
        headingController = new PIDController(4.0, 0, 0, 0.02); // Needs Calibration
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        drivePoseEstimator = mPoseEstimator;
    }

    public double update() {
        var aimingParameters = getAimingParameters();
        double output = headingController.calculate(drivePoseEstimator.getEstimatedPosition().getRotation().getRadians(), aimingParameters.driveHeading().getRadians());

        Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
        return output;
    }

    public AimingParameters getAimingParameters() {
        if (lastParamters != null) {
            return lastParamters;
        }

        Transform2d fieldToTarget = GeomUtil.toTransform2d(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
                .toTranslation2d());

        Pose2d fieldToPredictedPose = getPredictedPose(LOOK_AHEAD, LOOK_AHEAD);
        Pose2d fieldToPredictedPoseFixed = new Pose2d(fieldToPredictedPose.getTranslation(), new Rotation2d());

        Translation2d predictedVehicleToTargetTranslation =
            GeomUtil.inverse(fieldToPredictedPose).transformBy(fieldToTarget).getTranslation();
        Translation2d predictedVehicleFixedToTargetTranslation =
            GeomUtil.inverse(fieldToPredictedPoseFixed).transformBy(fieldToTarget).getTranslation();

        Rotation2d vehicleToGoalDirection = predictedVehicleToTargetTranslation.getAngle();

        Rotation2d targetRobotDirection = predictedVehicleFixedToTargetTranslation.getAngle();
        double targetDistance = predictedVehicleToTargetTranslation.getNorm();

        double feedVelocity = robotVelocity.dx * vehicleToGoalDirection.getSin() / targetDistance
                - robotVelocity.dy * vehicleToGoalDirection.getCos() / targetDistance;

        lastParamters = new AimingParameters(
                targetRobotDirection,
                new Rotation2d(
                        targetDistance - LauncherConstants.launcherOrigin.getX(),
                        FieldConstants.Speaker.centerSpeakerOpening.getZ()
                                - LauncherConstants.launcherOrigin.getY()
                                + HEIGHT_COMP),
                feedVelocity);
        Logger.recordOutput("AimingParameters/Direction", lastParamters.driveHeading);
        Logger.recordOutput("AimingParameters/launcherAngle", lastParamters.launcherAngle);
        Logger.recordOutput("AimingParameters/DriveFeedVelocityRadPerS", feedVelocity);
        return lastParamters;
    }

    /**
     * Predicts what our pose will be in the future. Allows separate translation and
     * rotation
     * lookaheads to account for varying latencies in the different measurements.
     *
     * @param translationLookaheadS The lookahead time for the translation of the
     *                              robot
     * @param rotationLookaheadS    The lookahead time for the rotation of the robot
     * @return The predicted pose.
     */
    public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
        return drivePoseEstimator.getEstimatedPosition()
                .exp(
                        new Twist2d(
                                robotVelocity.dx * translationLookaheadS,
                                robotVelocity.dy * translationLookaheadS,
                                robotVelocity.dtheta * rotationLookaheadS));
    }

}
