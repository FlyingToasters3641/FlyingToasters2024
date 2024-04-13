package frc.robot.commands;

import java.util.concurrent.locks.Condition;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotSystem;

public class LauncherCommands {

    private LauncherCommands() {
    }

    public static Command runFlywheelSpeed(Launcher m_launcher) {
        return Commands.runOnce(() -> {
            m_launcher.setFlywheelVelocity(1000);
        });
    }

    public static Command runRoller(Launcher m_launcher) {
        return Commands.run(() -> {
            m_launcher.setFeederVoltage(1.0);
        });
    }

    public static Command stopRoller(Launcher m_launcher) {
        return Commands.run(() -> {
            m_launcher.setFeederVoltage(0.0);
        });
    }

    public static Command stopLauncher(Launcher m_launcher) {
        return Commands.runOnce(() -> {
            m_launcher.stop();
        });
    }

    public static Command goToAngle(Launcher m_launcher, double angleDegrees) {
        return Commands.run(() -> {
            m_launcher.setAngleSetpoint(angleDegrees);
        });
    }

    // public static SequentialCommandGroup shootNote(Launcher m_launcher, Intake
    // m_intake, RobotSystem m_System) {

    // return new SequentialCommandGroup(
    // Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SHOOT)),
    // new WaitCommand(0.5),
    // Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    // }


    public static SequentialCommandGroup autoShootNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SHOOT)),
                new WaitCommand(0.5));
    }

    public static SequentialCommandGroup autoShootSubwoofer(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SUBWOOF_SHOOT)),
                new WaitCommand(0.5));
    }

    public static SequentialCommandGroup autoShoot8(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SHOOT_8)),
                new WaitCommand(0.5));
    }



    public static SequentialCommandGroup ampNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.AMP_SCORE)),
                new WaitCommand(0.5),
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }

    public static Command pitchJoystick(Launcher m_Launcher, DoubleSupplier joystick){
        return Commands.runOnce(() -> m_Launcher.setAngleSetpoint(joystick.getAsDouble() * 10.0));
    }

    public static ConditionalCommand Lob(Launcher m_launcher, RobotSystem m_System, Limelight m_limelight, DriveSubsystem drive) {
        return new ConditionalCommand(RedLob(m_launcher, m_System, m_limelight, drive), BlueLob(m_launcher, m_System, m_limelight, drive), () -> DriverStation.getAlliance().get() == Alliance.Red);
    }

    public static SequentialCommandGroup BlueLob(Launcher m_launcher, RobotSystem m_System, Limelight m_limelight, DriveSubsystem drive) {
        return new SequentialCommandGroup(  
            Commands.runOnce(() -> m_limelight.setPipeline(2)),
            Commands.runOnce(() -> drive.setLobGoal()),
            Commands.runOnce(() -> m_System.setGoalState(SystemState.AIM_LOB))
            );
    }
    
    public static SequentialCommandGroup RedLob(Launcher m_launcher, RobotSystem m_System, Limelight m_limelight, DriveSubsystem drive) {
        return new SequentialCommandGroup(  
            Commands.runOnce(() -> m_limelight.setPipeline(2)),
            Commands.runOnce(() -> drive.setLobGoal()),
            Commands.runOnce(() -> m_System.setGoalState(SystemState.AIM_LOB))
            );
    }

    public static ConditionalCommand Trap(Launcher m_launcher, RobotSystem m_System, PhotonCamera m_camera, DriveSubsystem drive){
        return new ConditionalCommand(null, BlueTrap(m_launcher, m_System, m_camera, drive), null);
    }

    public static SequentialCommandGroup BlueTrap(Launcher m_launcher, RobotSystem m_System, PhotonCamera m_camera, DriveSubsystem drive) {
        return new SequentialCommandGroup(  
            Commands.runOnce(() -> drive.setAimGoal()),
            new WaitCommand(0.3),
            Commands.runOnce(() -> m_System.setGoalState(SystemState.AIM_LOB)),
            new WaitCommand(1.00),
            Commands.runOnce(() -> m_System.setGoalState(SystemState.SHOOT_LOB)),
            new WaitCommand(0.5)
            );
    }
    
    public static ConditionalCommand EndLob(Launcher m_launcher, RobotSystem m_System, Limelight m_limelight, DriveSubsystem drive) {
        return new ConditionalCommand(RedLobEnd(m_launcher, m_System, m_limelight, drive), BlueLobEnd(m_launcher, m_System, m_limelight, drive), () -> DriverStation.getAlliance().get() == Alliance.Red);
    }
    
    public static SequentialCommandGroup BlueLobEnd(Launcher m_launcher, RobotSystem m_System, Limelight m_limelight, DriveSubsystem drive) {
        return new SequentialCommandGroup(  
            Commands.runOnce(() -> m_System.setGoalState(SystemState.SHOOT_LOB)),
            new WaitCommand(0.25),
            Commands.runOnce(() -> drive.clearLobGoal()),
            Commands.runOnce(() -> m_limelight.setPipeline(0)),
            Commands.runOnce(() -> m_System.setGoalState(SystemState.IDLE))
            );
    }
    
    public static SequentialCommandGroup RedLobEnd(Launcher m_launcher, RobotSystem m_System, Limelight m_limelight, DriveSubsystem drive) {
        return new SequentialCommandGroup(  
            Commands.runOnce(() -> m_System.setGoalState(SystemState.SHOOT_LOB)),
            new WaitCommand(0.25),
            Commands.runOnce(() -> drive.clearLobGoal()),
            Commands.runOnce(() -> m_limelight.setPipeline(1)),
            Commands.runOnce(() -> m_System.setGoalState(SystemState.IDLE))
            );
    }

    public static Command AutoAutoAim(Launcher m_launcher, Limelight m_limelight, DriveSubsystem drive) {
        return Commands.run(() -> {
           drive.setAutoAutoAim(true);
        }).until(() -> m_launcher.getLauncherNote() == false).andThen(
            Commands.runOnce(() -> drive.setAutoAutoAim(false)));
      }
    

    



}
