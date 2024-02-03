package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommands {

    private IntakeCommands() {}

    public static Command startFrontRollers(Intake m_intake) {
        return Commands.run(() -> 
           {m_intake.startFrontRollers();}
        );}
    
    public static Command stopFrontRollers(Intake m_intake) {
        return Commands.run(() -> 
            {m_intake.stopFrontRollers();}
            );}
            
    public static Command reverseFrontRollers(Intake m_intake) {
        return Commands.run(() -> 
            {m_intake.reverseFrontRollers();}
            );}
    
    public static Command runFrontSpeed(Intake m_intake, DoubleSupplier axis) {
        return Commands.run(() -> {m_intake.runFrontRollers(axis);});
    }
}
