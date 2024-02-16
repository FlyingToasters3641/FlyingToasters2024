package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.controllers.ShotController;

public class Launcher extends SubsystemBase {

  private LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  private ShotController shotController;


  private double angleSetpoint = 0.0;
  
  public final SysIdRoutine flywheelRoutine;

  public Launcher(LauncherIO io) {
    this.io = io;
    io.setBrakeMode(false, false, false, true);

    flywheelRoutine = new SysIdRoutine(new Config(null, null, null, (state) ->  Logger.recordOutput("Launcher/SysIdState", state.toString())), new Mechanism((voltage) -> {this.setFeederVoltage(voltage.in(Volts));}, (voltageState) -> {Logger.recordOutput("Launcher/SysIdVoltage", voltageState.toString());}, this));
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

  public double updateShot(SwerveDrivePoseEstimator poseEstimator) {
    shotController = new ShotController(poseEstimator);
    return shotController.updateAngle();
  }

  public boolean getNote(){
    return inputs.note;
  }

}
