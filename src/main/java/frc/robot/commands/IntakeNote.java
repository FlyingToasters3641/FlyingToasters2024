package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class IntakeNote extends Command{
    private final Intake intake;
    private final RobotSystem robotSys;
    private final Launcher launcher;
    private final LEDSubsystem ledSys;

    public IntakeNote(Intake m_intake, Launcher m_launcher, RobotSystem m_robotSys, LEDSubsystem m_LedSubsystem) {
    intake = m_intake;
    robotSys = m_robotSys;
    launcher = m_launcher;
    ledSys = m_LedSubsystem;

    addRequirements(m_intake, m_launcher, m_robotSys, m_LedSubsystem);
    
    }

    @Override
    public void initialize(){
        robotSys.setGoalState(SystemState.INTAKE);
    }

    @Override
    public void execute() {
    if (intake.getRearNote() == false) {
    IntakeCommands.rearIntakeNote(launcher, intake, robotSys);
    } else if (intake.getFrontNote() == false) {
    IntakeCommands.frontIntake(launcher, intake, robotSys, ledSys);
    }}

    
    @Override
    public boolean isFinished() {
        if (launcher.getLauncherNote() == false) {
            return true;
        } else {
            return false;
        }
    }
}
