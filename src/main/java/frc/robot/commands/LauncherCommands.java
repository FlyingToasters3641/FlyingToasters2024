package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher;

public class LauncherCommands {

    private LauncherCommands() {}
    
    
    public static Command runFlywheelSpeed(Launcher m_launcher){
        return Commands.runOnce(() -> {
            m_launcher.setFlywheelVelocity(10000);
        });
    }

    public static Command runRoller(Launcher m_launcher){
        return Commands.run(() -> {m_launcher.setFeederVoltage(1.0);});
    }

    public static Command stopRoller(Launcher m_launcher){
        return Commands.run(() -> {m_launcher.setFeederVoltage(0.0);});
    }


    public static Command stopLauncher(Launcher m_Launcher){
        return Commands.runOnce(() -> {
            m_Launcher.stop();
        });
    }
}