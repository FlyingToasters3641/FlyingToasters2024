
package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;

public class ShootNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RobotSystem robotSystem;
  private boolean endCommand;

  /**
   * Shoots the Note off the launcher
   *
   * @param m_System The subsystem used by this command.
   */
  public ShootNote(RobotSystem m_System) {
    robotSystem = m_System;
    addRequirements(m_System);
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