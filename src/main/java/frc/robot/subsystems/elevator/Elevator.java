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

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.elevator.*;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  ElevatorIOTalonFX ElevatorTalonFX = new ElevatorIOTalonFX();

  private double m_position;

  private Follower followerControl;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(false, false);

    leaderMotor = ElevatorIOTalonFX.leaderTalonFX;
    followerMotor = ElevatorIOTalonFX.followTalonFX;

    m_position = 0;

  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public void setElevatorPosition(DoubleSupplier position) {
    io.setPosition(position.getAsDouble());
    Logger.recordOutput("ElevatorPosition", position.getAsDouble());
    m_position = position.getAsDouble();
  }

  public void setElevatorPosition(double position) {
    io.setPosition(position);
    Logger.recordOutput("ElevatorPosition", position);
    m_position = position;
  }

  public double getElevatorPosition() {
    return io.getPosition();
  }

  public double elevatorAbsoluteEncoderPosition(){
    return io.getAbsolutePosition();
  }

  

}


