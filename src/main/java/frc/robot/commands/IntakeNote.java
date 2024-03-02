package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class IntakeNote extends Command {
    private final Intake intake;
    private final RobotSystem robotSys;
    private final Launcher launcher;

    public IntakeNote(Intake m_intake, Launcher m_launcher, RobotSystem m_robotSys) {
        intake = m_intake;
        robotSys = m_robotSys;
        launcher = m_launcher;

        addRequirements(m_intake, m_launcher, m_robotSys);
    }

    @Override
    public void execute() {
        robotSys.setGoalState(SystemState.INTAKE);
        if (intake.getRearNote() == false) {
            IntakeCommands.rearIntakeNote(launcher, intake, robotSys);
        } else if (intake.getFrontNote() == false) {
            robotSys.setGoalState(SystemState.FRONT_INTAKE);
            if (intake.getRearNote() == false) {
                IntakeCommands.rearIntakeNote(launcher, intake, robotSys);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (launcher.getLauncherNote() == false) {
            return true;
        } else {
            return false;
        }
    }
}
