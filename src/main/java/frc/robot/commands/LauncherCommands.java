package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher;

public class LauncherCommands {

    private LauncherCommands() {}
    
    
    public static Command runTopFlywheelSpeed(Launcher m_launcher, DoubleSupplier axis){
        return Commands.run(() -> {m_launcher.topFlywheelspeed(axis);});
    }

    public static Command runBottomFlywheelSpeed(Launcher m_launcher, DoubleSupplier axis){
        return Commands.run(() -> {m_launcher.bottomFlywheelspeed(axis);});
    }
}
