
package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;

/** An example command that uses an example subsystem. */
public class ShootSubwoofer extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RobotSystem robotSystem;
  private boolean endCommand;

  /**
   * Fast shot to the subwoofer
   *
   * @param m_System The subsystem used by this command.
   */
  public ShootSubwoofer(RobotSystem m_System) {
    robotSystem = m_System;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_System);
  }

  @Override
  public void initialize() {
    endCommand = false;
  }


  @Override
    public void execute() {
        robotSystem.setGoalState(SystemState.SUBWOOF_AIM);
          robotSystem.setGoalState(SystemState.SUBWOOF_SHOOT);
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