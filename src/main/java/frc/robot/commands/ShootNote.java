
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

/** An example command that uses an example subsystem. */
public class ShootNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Launcher launcher;
  private final Intake intake;
  private final RobotSystem robotSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_launcher The subsystem used by this command.
   */
  public ShootNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
    launcher = m_launcher;
    intake = m_intake;
    robotSystem = m_System;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher, m_intake, m_System);
  }


  @Override
    public void execute() {
        robotSystem.setGoalState(SystemState.AIM);
        if (launcher.atThreshold()) {
            robotSystem.setGoalState(SystemState.SHOOT);
            cancel();
        }
    }


}