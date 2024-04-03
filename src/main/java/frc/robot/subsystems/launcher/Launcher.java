package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controllers.ShotController;
import frc.robot.subsystems.Limelight;

public class Launcher extends SubsystemBase {

  private LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  private ShotController shotController;
  

  private double angleSetpoint = 0.0;

  public Launcher(LauncherIO io) {
    this.io = io;
    io.setBrakeMode(false, false, false, true);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Launcher/AbsolutePitch", inputs.launcherPosition);
    Logger.recordOutput("Launcher/PitchSensorDegrees", inputs.launcherPositionDegrees);
    Logger.recordOutput("Launcher/PitchMotorDegrees", inputs.pitchMotorSensorDegrees);
    Logger.recordOutput("Launcher/PitchSetpointDegrees", inputs.angleSetpointDegrees);
    Logger.recordOutput("Launcher/FeederVelocity", inputs.flywheelVelocity);
  }

  @Override
  public void simulationPeriodic() {
  }


  public void setFeederVoltage(double speed) {
    io.setFeederVoltage(speed);
  }

  public void setFlywheelVelocity(double rpm) {
    io.setFlywheelVelocity(rpm);
  }

  public void setTopFlywheelVelocity(double rpm) {
    io.setTopFlywheelVelocity(rpm);
  }

  public void setBottomFlywheelVelocity(double rpm) {
    io.setBottomFlywheelVelocity(rpm);
  }
  
  public Rotation2d getSetPoint() {
    return Rotation2d.fromRadians(angleSetpoint);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.launcherPositionDegrees);
  }

  public void stop(){
    io.stop();
  }

  public void setAngleSetpoint(double angleDegrees) {
    io.setAngleSetpoint(angleDegrees);
  }

  public double updateShot(Limelight m_Limelight, PhotonCamera m_vision) {
    shotController = new ShotController();
    return shotController.updateAngle(m_Limelight, m_vision);
  }

  public boolean getLauncherNote(){
    return inputs.note;
  }


  public boolean atShooterThreshold(){
    Logger.recordOutput("Launcher/AtShooterThreshold",io.atShooterThreshold());

    return io.atShooterThreshold();
  }
  
    public boolean withinPosition(double goToPosition){
        double threshold = 0.3;

        double launcherPosition = -inputs.launcherPositionDegrees;

            if (launcherPosition >= (goToPosition - threshold) 
        && launcherPosition <= (goToPosition + threshold)) {
            return true;
        } else {
            return false;
        }
    }
  
  public boolean atThreshold() {
    return io.atThreshold();
  }
}
