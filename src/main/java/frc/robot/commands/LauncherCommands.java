package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher;

public class LauncherCommands {

    private LauncherCommands() {}
    
    
    public static Command runFlywheelSpeed(Launcher m_launcher, DoubleSupplier axis){
        return Commands.run(() -> {
            m_launcher.topFlywheelspeed(axis); 
            m_launcher.bottomFlywheelspeed(axis);
        });
    }

    public static Command runRoller(Launcher m_launcher){
        return Commands.run(() -> {m_launcher.setFeederVoltage(1.0);});
    }

    public static Command stopRoller(Launcher m_launcher){
        return Commands.run(() -> {m_launcher.setFeederVoltage(0.0);});
    }

    public static Command runBottomFlywheelSpeed(Launcher m_launcher, DoubleSupplier axis){
        return Commands.run(() -> {m_launcher.bottomFlywheelspeed(axis);});
    }

    public static Command stopFlywheelBottom(Launcher m_Launcher){
        return Commands.run(() -> {m_Launcher.stopBottomFlywheel();});
    }

    public static Command stopFlywheel(Launcher m_Launcher){
        return Commands.run(() -> {m_Launcher.stopTopFlywheel();
                                   m_Launcher.stopBottomFlywheel();});
    }
}
