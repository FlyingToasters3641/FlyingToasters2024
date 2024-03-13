package frc.robot.subsystems;

import javax.swing.text.html.parser.Element;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.controllers.ShotController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;

public class RobotSystem extends SubsystemBase{
    public enum SystemState {
        IDLE,
        AIM,
        SHOOT,
        INTAKE,
        REAR_INTAKE,
        FRONT_INTAKE,
        FRONT_INTAKE_FEED,
        REVERSE_INTAKE,
        INTAKE_AND_SHOOT,
        STATION_INTAKE,
        AMP_AIM,
        AMP_SCORE,
        HUMAN_PLAYER,
        OUTTAKE,
        CLIMB_EXTEND,
        CLIMB_RETRACT,
        CLIMB_LOCK,
        SHOOT_TAPE,
        AIM_TAPE
    }

    public enum GamepieceState {
        NO_GAMEPIECE,
        HOLDING_GAMEPIECE
    }

    private SystemState currentState = SystemState.IDLE;
    private SystemState goalState = SystemState.IDLE;

    private final Launcher launcher;
    private final Intake intake;
    private final Elevator elevator;
    private final DriveSubsystem drive;

    private final ShotController shotController;

    public RobotSystem(Launcher m_launcher, Intake m_intake, Elevator m_elevator, DriveSubsystem m_drive) {
        launcher = m_launcher;
        intake = m_intake;
        elevator = m_elevator;
        drive = m_drive;
        shotController = new ShotController(m_drive.getPoseEstimator());
    }

    @Override 
    public void periodic(){
        switch(goalState) {
            case IDLE -> currentState = SystemState.IDLE;
            case AIM -> currentState = SystemState.AIM;
            case SHOOT -> currentState = SystemState.SHOOT;
            case INTAKE -> currentState = SystemState.INTAKE;
            case REAR_INTAKE -> currentState = SystemState.REAR_INTAKE;
            case FRONT_INTAKE -> currentState = SystemState.FRONT_INTAKE;
            case FRONT_INTAKE_FEED -> currentState = SystemState.FRONT_INTAKE_FEED;
            case REVERSE_INTAKE -> currentState = SystemState.REVERSE_INTAKE;
            case INTAKE_AND_SHOOT -> currentState = SystemState.INTAKE_AND_SHOOT;
            case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
            case AMP_AIM -> currentState = SystemState.AMP_AIM;
            case AMP_SCORE -> currentState = SystemState.AMP_SCORE;
            case HUMAN_PLAYER -> currentState = SystemState.HUMAN_PLAYER;
            case OUTTAKE -> currentState = SystemState.OUTTAKE;
            case CLIMB_EXTEND -> currentState = SystemState.CLIMB_EXTEND;
            case CLIMB_RETRACT -> currentState = SystemState.CLIMB_RETRACT;
            case CLIMB_LOCK -> currentState = SystemState.CLIMB_LOCK;
            case SHOOT_TAPE -> currentState = SystemState.SHOOT_TAPE;
            case AIM_TAPE -> currentState = SystemState.AIM_TAPE;
        }

        switch (currentState){
            case IDLE -> {
                launcher.setAngleSetpoint(LauncherConstants.IDLE);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case AIM -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case SHOOT -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.runFront();
                intake.runRear();
                elevator.setPosition(0.2);
            }
            case REAR_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.2);
                intake.stopFront();
                intake.runRear();
                elevator.setPosition(0.2);
            }
            case FRONT_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.5);
                intake.runFront();
                intake.reverseRear();
                elevator.setPosition(0.2);
            }
            case FRONT_INTAKE_FEED -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.15); //Only difference between REAR_INTAKE
                intake.stopFront();
                intake.runRearRollers(0.2);
                elevator.setPosition(0.2);
            }
            case REVERSE_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            } 
            case INTAKE_AND_SHOOT -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.runRear();
                elevator.setPosition(0.2);
            }
            case HUMAN_PLAYER -> {
                launcher.setAngleSetpoint(90);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_INTAKE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case AMP_AIM -> {
                elevator.setPosition(6.3);
                launcher.setAngleSetpoint(-35);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                
            }
            case AMP_SCORE -> {
                    elevator.setPosition(6.3);
                    launcher.setAngleSetpoint(-35);
                    launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP);
                    launcher.setFeederVoltage(0.3);
                    intake.stopFront();
                    intake.stopRear();
            }
            case OUTTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.reverseRear();
                elevator.setPosition(0.2);
            }
            case CLIMB_EXTEND -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(0.0);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(6);

            }
            case CLIMB_RETRACT -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(0.0);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.2);
            }
            case CLIMB_LOCK -> {
                launcher.setAngleSetpoint(100);
                launcher.setFlywheelVelocity(0.0);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
            }
            case SHOOT_TAPE -> {
                launcher.setAngleSetpoint(28);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
            }
            case AIM_TAPE -> {
                launcher.setAngleSetpoint(28);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
            }
            }
        }
    

    public void setGoalState(SystemState m_goalState){
        goalState = m_goalState;
    }

    public Command scoreAmp(Launcher m_Launcher, Elevator m_Elevator){
        return Commands.runOnce(() -> new SequentialCommandGroup(
            Commands.runOnce(() -> m_Elevator.setPosition(5.4)),
            Commands.runOnce(() -> m_Launcher.setAngleSetpoint(-5)),
            Commands.runOnce(() -> m_Launcher.setFeederVoltage(1)),
            Commands.runOnce(() -> m_Launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP))
            )
        );
    }

    public Command moveWrist(Launcher m_Launcher, double AngleSetpoint) {
        return Commands.runOnce(() -> m_Launcher.setAngleSetpoint(AngleSetpoint));
    }
}
