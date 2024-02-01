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

  private TalonFX leaderMotor = ElevatorIOTalonFX.leaderTalonFX;
  private TalonFX followerMotor = ElevatorIOTalonFX.followTalonFX;


  

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(false, false);

    followerMotor.setControl(ControlRequest.Follower(leaderMotor));
    followerMotor.setInverted(InvertType.FollowMaster);
  }



  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
