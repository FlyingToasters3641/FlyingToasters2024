package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  private LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

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

}
