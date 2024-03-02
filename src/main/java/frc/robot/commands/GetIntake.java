
package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeStates;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

/** An example command that uses an example subsystem. */
public class GetIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Launcher launcher;
  private final RobotSystem robotSystem;
  private final Intake intake;
  private final IntakeStates intakeSys;
  private boolean endCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_launcher The subsystem used by this command.
   */
  public GetIntake(Launcher m_launcher, RobotSystem m_System, Intake m_intake, IntakeStates m_intakeSys) {
    launcher = m_launcher;
    robotSystem = m_System;
    intake = m_intake;
    intakeSys = m_intakeSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher, m_System, m_intake, m_intakeSys);
  }

  @Override
  public void initialize() {
    endCommand = false;
  }


  @Override
    public void execute() {
      intakeSys.setIntakeGoalState(IntakeStates.IntakeSystem.INTAKE);
      if (intake.getrearIntake() == true) {
        IntakeCommands.rearIntakeNote(launcher, intake, robotSystem, intakeSys);
      } else if (intake.getfrontIntake() == false) {
        IntakeCommands.frontIntakeNote(launcher, intake, robotSystem, intakeSys);
      }
    }

  @Override
  public boolean isFinished() {
    if (endCommand == true) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }


}