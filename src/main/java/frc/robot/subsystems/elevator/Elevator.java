package frc.robot.subsystems.elevator;

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

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  ElevatorIOTalonFX ElevatorTalonFX = new ElevatorIOTalonFX();

  private Follower followerControl;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(false, false);

    leaderMotor = ElevatorIOTalonFX.leaderTalonFX;
    followerMotor = ElevatorIOTalonFX.followTalonFX;

    
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

   public void elevatorPosition(DoubleSupplier position){
    io.setPosition(position.getAsDouble());
  }
}

enum ElevatorPos {

  STORED_POSITION(0, 0, false, false),
  AMP_POSITION(0, 0, false, false), // -34
  CHAIN_POSITION(0, 0, true, false), // -23
  TRAP_POSITION(0, 0, true, false), // 131
  PLAYER_STATION_POSITION(0, 0, false, false),
  SPEAKER_POSITION(0, 0, false, false); // 148

  private double elevatorPosition;
  private double wristAngle;
  private boolean runLauncher;
  private boolean isFront;

  private ElevatorPos(double elevatorPosition, double wristAngle, boolean runLauncher, boolean isFront) {
    this.elevatorPosition = elevatorPosition;
    this.wristAngle = wristAngle;
    this.runLauncher = runLauncher;
    this.isFront = isFront;
  }

  public double getElevatorPosition() {
    return elevatorPosition;
  }

  public double getWristAngle() {
    return wristAngle;
  }

  public boolean getRunLauncher() {
    return runLauncher;
  }

  public boolean getIsFront() {
    return isFront;
  }

 
}
