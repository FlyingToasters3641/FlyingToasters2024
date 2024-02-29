package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        FRONT_INTAKE,
        REVERSE_INTAKE,
        STATION_INTAKE,
        AMP_AIM,
        AMP_SCORE,
        HUMAN_PLAYER,
        SHOOT_WITH_ELEVATOR //Example enum which uses the elevator in it.
    }

    public enum GamepieceState {
        NO_GAMEPIECE,
        HOLDING_GAMEPIECE
    }

    private SystemState currentState = SystemState.IDLE;
    private SystemState goalState = SystemState.IDLE;

    private final Launcher launcher;
    private final Intake intake;
    private final DriveSubsystem drive;
    private final Elevator elevator;

    private final ShotController shotController;

    public RobotSystem(Launcher m_launcher, Intake m_intake, DriveSubsystem m_drive, Elevator m_elevator) {
        launcher = m_launcher;
        intake = m_intake;
        drive = m_drive;
        elevator = m_elevator;
        shotController = new ShotController(m_drive.getPoseEstimator());
    }

    @Override 
    public void periodic(){
        switch(goalState) {
            case IDLE -> currentState = SystemState.IDLE;
            case AIM -> currentState = SystemState.AIM;
            case SHOOT -> currentState = SystemState.SHOOT;
            case INTAKE -> currentState = SystemState.INTAKE;
            case FRONT_INTAKE -> currentState = SystemState.FRONT_INTAKE;
            case REVERSE_INTAKE -> currentState = SystemState.REVERSE_INTAKE;
            case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
            case AMP_AIM -> currentState = SystemState.AMP_AIM;
            case AMP_SCORE -> currentState = SystemState.AMP_SCORE;
            case HUMAN_PLAYER -> currentState = SystemState.HUMAN_PLAYER;
            case SHOOT_WITH_ELEVATOR -> currentState = SystemState.SHOOT_WITH_ELEVATOR;
        }

        switch (currentState){
            case IDLE -> {
                launcher.setAngleSetpoint(LauncherConstants.IDLE);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
            }
            case AIM -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
            }
            case SHOOT -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear();
            }
            case INTAKE -> {
                launcher.setAngleSetpoint(30);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.15);
                intake.stopFront();
                intake.runRear();
            }
            case FRONT_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_INTAKE);
                launcher.setFeederVoltage(0.15);
                intake.runFront();
                intake.stopRear();
            }
            case REVERSE_INTAKE -> {
                launcher.setAngleSetpoint(30);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.5);
                intake.stopFront();
                intake.stopRear();
            } 
            case HUMAN_PLAYER -> {
                launcher.setAngleSetpoint(90);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_INTAKE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.stopRear();
            }
            case SHOOT_WITH_ELEVATOR -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(35, 0); //testing the elevator commands
            }
            }
        }
    

    public void setGoalState(SystemState m_goalState){
        goalState = m_goalState;
    }
}
