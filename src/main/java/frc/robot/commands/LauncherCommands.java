package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotSystem;

public class LauncherCommands {

    private LauncherCommands() {
    }

    /** Runs the Flywheel at a high speed for testing  
    */
    public static Command runFlywheelSpeed(Launcher m_launcher) {
        return Commands.runOnce(() -> {
            m_launcher.setFlywheelVelocity(1000);
        });
    }

    /** Runs the rollers at full speed */
    public static Command runRoller(Launcher m_launcher) {
        return Commands.run(() -> {
            m_launcher.setFeederVoltage(1.0);
        });
    }

    /* Stops the rollers */
    public static Command stopRoller(Launcher m_launcher) {
        return Commands.run(() -> {
            m_launcher.setFeederVoltage(0.0);
        });
    }

    /** Stops the launcher using the subsystem */
    public static Command stopLauncher(Launcher m_launcher) {
        return Commands.runOnce(() -> {
            m_launcher.stop();
        });
    }

    /** Sets the launcher to go a setpoint */
    public static Command goToAngle(Launcher m_launcher, double angleDegrees) {
        return Commands.run(() -> {
            m_launcher.setAngleSetpoint(angleDegrees);
        });
    }

    /** Different shooting logic for autonomous mode */
    public static SequentialCommandGroup autoShootNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SHOOT)),
                new WaitCommand(0.5));
    }

    /** Different subwoofer shot for autonomous mode */
    public static SequentialCommandGroup autoShootSubwoofer(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SUBWOOF_SHOOT)),
                new WaitCommand(0.5));
    }

    /** Shoots a shot at an angle setpoint of 8 degrees */
    public static SequentialCommandGroup autoShoot8(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.SHOOT_8)),
                new WaitCommand(0.5));
    }

    /**  Runs condtional logic through the LauncherCommands logic to give an accurate lob shot for the right part of the field */
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

    /** Failed autoaim during autonomous mode */
    public static Command AutoAutoAim(Launcher m_launcher, Limelight m_limelight, DriveSubsystem drive) {
        return Commands.run(() -> {
           drive.setAutoAutoAim(true);
        }).until(() -> m_launcher.getLauncherNote() == false).andThen(
            Commands.runOnce(() -> drive.setAutoAutoAim(false)));
      }

}
