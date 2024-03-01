package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controllers.ShotController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;

public class IntakeStates extends SubsystemBase{
    public enum IntakeSystem {
        IDLE,
        INTAKE,
        FRONT_INTAKE,
        REVERSE_INTAKE,
        REAR_INTAKE,
        LAUNCHER_FEED

    }

    public enum GamepieceState {
        NO_GAMEPIECE,
        HOLDING_GAMEPIECE
    }

    private IntakeSystem currentState = IntakeSystem.IDLE;
    private IntakeSystem goalState = IntakeSystem.IDLE;

    private final Launcher launcher;
    private final Intake intake;
    private final DriveSubsystem drive;

    private final ShotController shotController;

    public IntakeStates(Launcher m_launcher, Intake m_intake, DriveSubsystem m_drive) {
        launcher = m_launcher;
        intake = m_intake;
        drive = m_drive;
        shotController = new ShotController(m_drive.getPoseEstimator());
    }

    @Override 
    public void periodic(){
        switch(goalState) {
            case IDLE -> currentState = IntakeSystem.IDLE;
            case INTAKE -> currentState = IntakeSystem.INTAKE;
            case FRONT_INTAKE -> currentState = IntakeSystem.FRONT_INTAKE;
            case REVERSE_INTAKE -> currentState = IntakeSystem.REVERSE_INTAKE;
            case REAR_INTAKE -> currentState = IntakeSystem.REVERSE_INTAKE;
            case  LAUNCHER_FEED -> currentState = IntakeSystem.REVERSE_INTAKE;
            
        }

        switch (currentState){
            case IDLE -> {
                launcher.setAngleSetpoint(LauncherConstants.IDLE);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
            }
            case INTAKE -> {
                launcher.setAngleSetpoint(30);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.15);
                intake.runFront();
                intake.runRear();
            }
            case FRONT_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.runFront();
                intake.stopRear();
            }
            case REAR_INTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.reverseRearRollers();
                intake.stopFront();
            }  
            case LAUNCHER_FEED -> {
                launcher.setAngleSetpoint(66);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP);
                launcher.setFeederVoltage(0.4);
                intake.stopFront();
                intake.stopRear();
            }
            case REVERSE_INTAKE -> {
                launcher.setAngleSetpoint(30);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(-0.5);
                intake.stopFront();
                intake.stopRear();
            }  
            }
        }
    

    public void setIntakeGoalState(IntakeSystem m_goalState){
        goalState = m_goalState;
    }
}
