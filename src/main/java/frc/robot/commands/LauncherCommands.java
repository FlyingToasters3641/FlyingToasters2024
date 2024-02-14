package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.RobotSystem.SystemState;
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

    public static Command intakeNote(Launcher m_launcher, RobotSystem m_robotSystem) {
        return Commands.runOnce(() -> {
            if(m_launcher.getNote() == false) {
                m_robotSystem.setGoalState(SystemState.INTAKE);
            } else {
                
            }

        });

    }
}
