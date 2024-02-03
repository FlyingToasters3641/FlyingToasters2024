package frc.robot.subsystems.launcher;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.elevator.*;

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
  }

  @Override
  public void simulationPeriodic() {
  }

   public void wristPosition(DoubleSupplier position){
    io.setPosition(position.getAsDouble());
  }
}
