package frc.robot.subsystems.launcher;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Launcher extends SubsystemBase {

  private LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  LauncherIOTalonFX LauncherTalonFX = new LauncherIOTalonFX();

  private Follower followerControl;

  public Launcher(LauncherIO io) {
    this.io = io;
    io.setBrakeMode(false, false, false, false);
    
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  @Override
  public void simulationPeriodic() {
  }

   public void wristPosition(DoubleSupplier position){
    io.setPosition(position.getAsDouble());
  }

  public void topFlywheelspeed(DoubleSupplier speed) {
    io.setTopFlywheelRollers(speed.getAsDouble());
  }
  
  public void bottomFlywheelspeed(DoubleSupplier speed) {
    io.setBottomFlywheelRollers(speed.getAsDouble());
  }

  public void stopBottomFlywheel() {
  }

  public void stopTopFlywheel() {
  }

  public void setFeederVoltage(double speed) {
    io.setFeederVoltage(speed);
  }
 public Translation3d calcTrajectory(Translation3d robotTrajectory, Translation3d shooterTrajectory) {
    return io.calcTrajectory(robotTrajectory, shooterTrajectory);
  }
  
}
