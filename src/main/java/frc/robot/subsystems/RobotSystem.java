package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controllers.ShotController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;

public class RobotSystem extends SubsystemBase{
    public enum SystemState {
        IDLE,
        EXTERNAL_INTAKE,
        AIM,
        SHOOT,
        SUBWOOF_AIM,
        SUBWOOF_SHOOT,
        INTAKE,
        REAR_INTAKE,
        FRONT_INTAKE,
        FRONT_INTAKE_FEED,
        REVERSE_INTAKE,
        INTAKE_AND_SHOOT,
        AMP_AIM,
        AMP_SCORE,
        HUMAN_PLAYER,
        OUTTAKE,
        FRONT_OUTTAKE,
        CLIMB_EXTEND,
        CLIMB_RETRACT,
        CLIMB_LOCK,
        SHOOT_LOB,
        AIM_LOB,
        AMP_MIDDLE,
        AMP_UP,
        TRAP_AIM,
        TRAP_SHOOT,
        TRAP_UP,
        TRAP_DOWN,
        SHOOT_8,
        AIM_8
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
    private final Limelight limelight;
    private final PhotonCamera vision;

    private final ShotController shotController;

    public RobotSystem(Launcher m_launcher, Intake m_intake, Elevator m_elevator, Limelight m_Limelight, PhotonCamera m_vision) {
        launcher = m_launcher;
        intake = m_intake;
        elevator = m_elevator;
        limelight = m_Limelight;
        vision = m_vision;
        shotController = new ShotController();
    }

    @Override 
    public void periodic(){
        switch(goalState) {
            case IDLE -> currentState = SystemState.IDLE;
            case EXTERNAL_INTAKE -> currentState = SystemState.EXTERNAL_INTAKE;
            case AIM -> currentState = SystemState.AIM;
            case SHOOT -> currentState = SystemState.SHOOT;
            case SUBWOOF_AIM -> currentState = SystemState.SUBWOOF_AIM;
            case SUBWOOF_SHOOT -> currentState = SystemState.SUBWOOF_SHOOT;
            case INTAKE -> currentState = SystemState.INTAKE;
            case REAR_INTAKE -> currentState = SystemState.REAR_INTAKE;
            case FRONT_INTAKE -> currentState = SystemState.FRONT_INTAKE;
            case FRONT_INTAKE_FEED -> currentState = SystemState.FRONT_INTAKE_FEED;
            case REVERSE_INTAKE -> currentState = SystemState.REVERSE_INTAKE;
            case INTAKE_AND_SHOOT -> currentState = SystemState.INTAKE_AND_SHOOT;
            case AMP_AIM -> currentState = SystemState.AMP_AIM;
            case AMP_SCORE -> currentState = SystemState.AMP_SCORE;
            case HUMAN_PLAYER -> currentState = SystemState.HUMAN_PLAYER;
            case OUTTAKE -> currentState = SystemState.OUTTAKE;
            case FRONT_OUTTAKE -> currentState = SystemState.FRONT_OUTTAKE;
            case CLIMB_EXTEND -> currentState = SystemState.CLIMB_EXTEND;
            case CLIMB_RETRACT -> currentState = SystemState.CLIMB_RETRACT;
            case CLIMB_LOCK -> currentState = SystemState.CLIMB_LOCK;
            case SHOOT_LOB -> currentState = SystemState.SHOOT_LOB;
            case AIM_LOB -> currentState = SystemState.AIM_LOB;
            case AMP_MIDDLE -> currentState = SystemState.AMP_MIDDLE;
            case AMP_UP -> currentState = SystemState.AMP_UP;
            case TRAP_AIM -> currentState = SystemState.TRAP_AIM;
            case TRAP_SHOOT -> currentState = SystemState.TRAP_SHOOT;
            case TRAP_UP -> currentState = SystemState.TRAP_UP;
            case TRAP_DOWN -> currentState = SystemState.TRAP_DOWN;
            case SHOOT_8 -> currentState = SystemState.SHOOT_8;
            case AIM_8 -> currentState = SystemState.AIM_8;
        }

        switch (currentState){
            case IDLE -> {
                launcher.setAngleSetpoint(LauncherConstants.IDLE);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                launcher.setBlower(false);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case EXTERNAL_INTAKE -> {
                launcher.setAngleSetpoint(LauncherConstants.IDLE);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(1.0);
            }
            case AIM -> {
                launcher.setAngleSetpoint(shotController.updateAngle(limelight, vision));
                launcher.setFlywheelVelocity(shotController.updateRollers(limelight, vision));
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.runRear();
                elevator.setPosition(0.2);
            }
            case SHOOT -> {
                launcher.setAngleSetpoint(shotController.updateAngle(limelight, vision));
                launcher.setFlywheelVelocity(shotController.updateRollers(limelight, vision));
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case SUBWOOF_AIM -> {
                launcher.setAngleSetpoint(50);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_SUBWOOFER);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case SUBWOOF_SHOOT -> {
                launcher.setAngleSetpoint(50);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_SUBWOOFER);
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
                launcher.setAngleSetpoint(shotController.updateAngle(limelight, vision));
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.runRearMax();
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
            }
            case AMP_MIDDLE -> {
                elevator.setPosition(1.0);
                launcher.setAngleSetpoint(90);
                launcher.setFlywheelVelocity(7.0);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.stopRear();
            }
            case AMP_SCORE -> {
                elevator.setPosition(6.3);
                launcher.setAngleSetpoint(-35);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_AMP);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
            }
            case AMP_UP -> {
                elevator.setPosition(3);
                launcher.setAngleSetpoint(0.0);
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
            } case FRONT_OUTTAKE -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_IDLE);
                launcher.setFeederVoltage(0.5);
                intake.reverseFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case CLIMB_EXTEND -> {
                launcher.setAngleSetpoint(0);
                launcher.setFlywheelVelocity(0.0);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(6.4);

            }
            case CLIMB_RETRACT -> {
                launcher.setAngleSetpoint(80);
                launcher.setFlywheelVelocity(0.0);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
            }
            case CLIMB_LOCK -> {
                launcher.setAngleSetpoint(100);
                launcher.setFlywheelVelocity(0.0);
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
            }
            case SHOOT_LOB -> {
                launcher.setAngleSetpoint(shotController.updateLobbedAngle(limelight));
                launcher.setFlywheelVelocity(shotController.updateLobbedRollers(limelight));
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
                
            }
            case AIM_LOB -> {
                launcher.setAngleSetpoint(shotController.updateLobbedAngle(limelight));
                launcher.setFlywheelVelocity(shotController.updateLobbedRollers(limelight));
                launcher.setFeederVoltage(0);
                intake.stopFront();
                intake.stopRear(); 
                elevator.setPosition(0.1);
            }
            case TRAP_AIM -> {
                elevator.setPosition(0.2);
                launcher.setAngleSetpoint(60);
                launcher.setTopFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_TOP_TRAP);
                launcher.setBottomFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_BOTTOM_TRAP);
                launcher.setFeederVoltage(0.0);
                launcher.setBlower(true);
                intake.stopFront();
            }
            case TRAP_SHOOT -> {
                elevator.setPosition(0.2);
                launcher.setAngleSetpoint(60);
                launcher.setTopFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_TOP_TRAP);
                launcher.setBottomFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_BOTTOM_TRAP);
                launcher.setFeederVoltage(1.0);
                launcher.setBlower(true);
                intake.stopFront();
            } 
            case TRAP_UP -> {
                elevator.setPosition(0.2);
                launcher.setAngleSetpoint(0);
                launcher.setTopFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_TOP_TRAP);
                launcher.setBottomFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_BOTTOM_TRAP);
                launcher.setFeederVoltage(0.0);
                launcher.setBlower(true);
                intake.stopFront();
            }
            case TRAP_DOWN -> {
                elevator.setPosition(0.2);
                launcher.setAngleSetpoint(60);
                launcher.setFlywheelVelocity(LauncherConstants.IDLE);
                launcher.setFeederVoltage(0.0);
                launcher.setBlower(false);
                intake.stopFront();
            }
            case SHOOT_8 -> {
                launcher.setAngleSetpoint(8);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(1.0);
                intake.stopFront();
                intake.stopRear();
                elevator.setPosition(0.2);
            }
            case AIM_8 -> {
                launcher.setAngleSetpoint(8);
                launcher.setFlywheelVelocity(LauncherConstants.FLYWHEEL_RPM_DEFAULT);
                launcher.setFeederVoltage(0.0);
                intake.stopFront();
                intake.runRear();
                elevator.setPosition(0.2);
            }
            }
        }
    

    public void setGoalState(SystemState m_goalState){
        goalState = m_goalState;
    }


    public Command moveWrist(Launcher m_Launcher, double AngleSetpoint) {
        return Commands.runOnce(() -> m_Launcher.setAngleSetpoint(AngleSetpoint));
    }

    public SystemState getGoalState(){
        return goalState;
    }

    public Command setPipeline(Limelight m_Limelight, int pipeline) {
        return Commands.runOnce(() -> m_Limelight.setPipeline(pipeline));
    }

    
}
