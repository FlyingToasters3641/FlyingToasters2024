
package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

/** An example command that uses an example subsystem. */
public class ShootNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Launcher launcher;
  private final RobotSystem robotSystem;
  private boolean endCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_launcher The subsystem used by this command.
   */
  public ShootNote(Launcher m_launcher, RobotSystem m_System) {
    launcher = m_launcher;
    robotSystem = m_System;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher, m_System);
  }

  @Override
  public void initialize() {
    endCommand = false;
  }


  @Override
    public void execute() {
        robotSystem.setGoalState(SystemState.AIM);
          robotSystem.setGoalState(SystemState.SHOOT);
          endCommand = true;
        Logger.recordOutput("Launcher/endcommand", endCommand);
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