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
        REVERSE_INTAKE,
        STATION_INTAKE,
        AMP_AIM,
        AMP_SCORE,
        HUMAN_PLAYER,
        REAL_REVERSE_INTAKE
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
            case REVERSE_INTAKE -> currentState = SystemState.REVERSE_INTAKE;
            case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
            case AMP_AIM -> currentState = SystemState.AMP_AIM;
            case AMP_SCORE -> currentState = SystemState.AMP_SCORE;
            case HUMAN_PLAYER -> currentState = SystemState.HUMAN_PLAYER;
            case REAL_REVERSE_INTAKE -> currentState = SystemState.REAL_REVERSE_INTAKE;
        }

        switch (currentState){
            case IDLE -> {
                launcher.setAngleSetpoint(LauncherConstants.IDLE);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.1);
            }
            case AIM -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.1);
            }
            case SHOOT -> {
                launcher.setAngleSetpoint(shotController.updateAngle());
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.1);
            }
            case INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.runFront();
                intake.runRear();
                elevator.setPosition(0.1);
            }
            case REAR_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.2);
                intake.stopFront();
                intake.runRear();
                elevator.setPosition(0.1);
            }
            case FRONT_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.5);
                intake.runFront();
                intake.reverseRear();
                elevator.setPosition(0.1);
            }
            case REVERSE_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.1);
            } 
            case HUMAN_PLAYER -> {
                launcher.setAngleSetpoint(90);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_INTAKE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.1);
            }
            case AMP_AIM -> {
                if (launcher.getLauncherNote() == false){
                    elevator.setPosition(6.1);
            
                }else{
                    setGoalState(SystemState.IDLE);
                }
                
                launcher.setAngleSetpoint(0.0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                
            }
            case AMP_SCORE -> {
                if (launcher.getLauncherNote() == false){
                    elevator.setPosition(6.3);
                    launcher.setAngleSetpoint(-35);
                    launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP);
                    launcher.setFeederVoltage(0.1);
                    intake.stopFront();
                    intake.stopRear();
                }else{
                    setGoalState(SystemState.IDLE);
                }
            }
            case REAL_REVERSE_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.2);
                intake.stopFront();
                intake.reverseRear();
                elevator.setPosition(0);
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
