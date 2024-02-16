package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.RobotSystem;

public class LauncherCommands {

    private LauncherCommands() {}
    
    
    public static Command runFlywheelSpeed(Launcher m_launcher){
        return Commands.runOnce(() -> {
            m_launcher.setFlywheelVelocity(1000);
        });
    }

    public static Command runRoller(Launcher m_launcher){
        return Commands.run(() -> {m_launcher.setFeederVoltage(1.0);});
    }

    public static Command stopRoller(Launcher m_launcher){
        return Commands.run(() -> {m_launcher.setFeederVoltage(0.0);});
    }


    public static Command stopLauncher(Launcher m_launcher){
        return Commands.runOnce(() -> {
            m_launcher.stop();
        });
    }

    public static Command goToAngle(Launcher m_launcher, double angleDegrees){
        return Commands.run(() -> {
            m_launcher.setAngleSetpoint(angleDegrees);
        });
    }

    public static Command rearIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.INTAKE);
        }).until(() -> m_launcher.getNote() == false).andThen(
            new SequentialCommandGroup(
                Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
                Commands.waitSeconds(0.1))).andThen(
                    Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }
    
    public static Command frontIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.FRONT_INTAKE);
        }).until(() -> m_launcher.getNote() == false).andThen(
            new SequentialCommandGroup(
                Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
                Commands.waitSeconds(0.1))).andThen(
                    Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }

    public static SequentialCommandGroup shootNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return new SequentialCommandGroup(
          Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.SHOOT)),
          new WaitCommand(0.5),
          Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.IDLE))
            );
    }
    
    public static Command humanIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.HUMAN_PLAYER);
        }).until(() -> m_launcher.getNote() == false).andThen(
            new SequentialCommandGroup(
                Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
                Commands.waitSeconds(0.1))).andThen(
                    Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }


}
