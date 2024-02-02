package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.elevator.*;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(false, false);

    leaderMotor = ElevatorIOTalonFX.leaderTalonFX;
    followerMotor = ElevatorIOTalonFX.followTalonFX;

    followerMotor.setControl(ControlRequest.Follower);

  }


  

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

enum ElevatorPos {

  STORED_POSITION(0, 0, false),
  AMP_POSITION(0, 0, false), // -34
  CHAIN_POSITION(0, 0, true), // -23
  TRAP_POSITION(0, 0, true), // 131
  PLAUER_STATION_POSITION(0, 0, false),
  SPEAKER_POSITION(0, 0, false); // 148

  private double elevatorPosition;
  private double wristAngle;
  private boolean runLauncher;

  private ElevatorPos(double elevatorPosition, double wristAngle, boolean runLauncher) {
    this.elevatorPosition = elevatorPosition;
    this.wristAngle = wristAngle;
    this.runLauncher = runLauncher;
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
}
