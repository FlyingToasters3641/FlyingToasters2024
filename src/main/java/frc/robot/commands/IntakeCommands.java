package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

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
    
}
