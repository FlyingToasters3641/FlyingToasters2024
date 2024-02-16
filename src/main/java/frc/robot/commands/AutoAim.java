package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class AutoAim extends Command{
    protected final DriveSubsystem drive;
  protected final PoseEstimator poseEstimator;
    public record AimingParameters(
      Rotation2d driveHeading, Rotation2d launcherAngle, double driveFeedVelocity, double distanceToTarget) {}
    private PIDController headingController;
    private AimingParameters latestParameters = null;
    private static final double lookahead = 0.0;

    private static double shotHeightCompensation = 0.0;

    private Pose2d currentEsitmatedPose = null;

    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    public AutoAim(DriveSubsystem drive, SwerveDrivePoseEstimator poseEstimator)

    @Override
    public void initialize(){

    
    }

    @Override
    public void execute(){
        Transform2d fieldToTarget =
        GeomUtil.toTransform2d(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).toTranslation2d());
    Pose2d fieldToPredictedVehicle = getPredictedPose(lookahead, lookahead);
    Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

    Translation2d predictedVehicleToTargetTranslation =
        GeomUtil.inverse(fieldToPredictedVehicle).transformBy(fieldToTarget).getTranslation();
    Translation2d predictedVehicleFixedToTargetTranslation =
        GeomUtil.inverse(fieldToPredictedVehicleFixed).transformBy(fieldToTarget).getTranslation();

    Rotation2d vehicleToGoalDirection = predictedVehicleToTargetTranslation.getAngle();

    Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
    double targetDistance = predictedVehicleToTargetTranslation.getNorm();

    double feedVelocity =
        robotVelocity.vxMetersPerSecond * vehicleToGoalDirection.getSin() / targetDistance
            - robotVelocity.vyMetersPerSecond * vehicleToGoalDirection.getCos() / targetDistance;

    latestParameters =
        new AimingParameters(
            targetVehicleDirection,
            new Rotation2d(
                targetDistance - LauncherConstants.launcherOrigin.getX(),
                FieldConstants.Speaker.centerSpeakerOpening.getZ()
                    - LauncherConstants.launcherOrigin.getY()
                    + shotHeightCompensation),
            feedVelocity,
            targetDistance);
    Logger.recordOutput("RobotState/AimingParameters/Direction", latestParameters.driveHeading);
    Logger.recordOutput("RobotState/AimingParameters/ArmAngle", latestParameters.launcherAngle);
    Logger.recordOutput("RobotState/AimingParameters/DriveFeedVelocityRadPerS", feedVelocity);
    }

    /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    return currentEsitmatedPose
        .exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * translationLookaheadS,
                robotVelocity.vyMetersPerSecond * translationLookaheadS,
                robotVelocity.omegaRadiansPerSecond * rotationLookaheadS));
  }
}
    



