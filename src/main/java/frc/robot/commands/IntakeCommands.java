package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class IntakeCommands {

    private IntakeCommands() {}

    /** Runs the front intake rollers at the speed specified
     * 
     * @param axis double supplied to run rollers. 1 is max speed.
     * @param m_intake intake subsystem
     *
     */
    public static Command runFrontSpeed(Intake m_intake, DoubleSupplier axis) {
      return Commands.run(() -> {
        m_intake.runFrontRollers(axis);
      });
    }

    
    /** Runs the rear intake rollers at the speed specified
     * 
     * @param axis double supplied to run rollers. 1 is max speed.
     * @param m_intake intake subsystem
     *
     */
    public static Command runRearSpeed(Intake m_intake, double axis) {
      return Commands.run(() -> {
        m_intake.runRearRollers(axis);
      });
    }

    
    /** Stops the front intake rollers */
    public static Command IN_StopFront(Intake m_intake) {
      return Commands.run(() -> {
        m_intake.stopFront();
      });
    }

    /** Stops the rear intake rollers */
    public static Command stopRear(Intake m_intake) {
      return Commands.run(() -> {
        m_intake.stopRear();
      });
    }

    /** Runs the rear intake rollers without spilling out the note
     * 
     * @param m_launcher Launcher subsystem
     * @param m_intake Intake subsystem
     * @param m_System RobotSystem subsystem
     * @return SequentialCommandGroup that runs logic to not spit out the note
     */
    public static Command rearIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.REAR_INTAKE);
        }).until(() -> m_launcher.getLauncherNote() == false).andThen(
            new SequentialCommandGroup(
                Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
                Commands.waitSeconds(0.05))).andThen(
                    Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }
    
    /** Runs the front intake rollers without spilling out the note
     * 
     * @param m_launcher Launcher subsystem
     * @param m_intake Intake subsystem
     * @param m_System RobotSystem subsystem
     * @return SequentialCommandGroup that runs logic to not spit out the note
     */
    public static Command frontIntakeFeed(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.FRONT_INTAKE_FEED);
        }).until(() -> m_launcher.getLauncherNote() == false).andThen(
            new SequentialCommandGroup(
                Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
                Commands.waitSeconds(0.05))).andThen(
                    Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }

    
    /** Outtakes the rear note
     * 
     * @param m_launcher Launcher subsystem
     * @param m_intake Intake subsystem
     * @param m_System RobotSystem subsystem
     * @return the command
     */
    public static Command rearOutakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.OUTTAKE);
        });
    }

  
    /** logic for the front intake note
     * 
     * @param m_launcher Launcher subsystem
     * @param m_intake Intake subsystem
     * @param m_System RobotSystem subsystem
     * @return the command
     */
  public static Command frontIntake(Launcher m_launcher, Intake m_intake, RobotSystem m_robotSystem){
    return Commands.run(() -> m_robotSystem.setGoalState(SystemState.FRONT_INTAKE))
    .until(() -> m_intake.getRearNote() == false).andThen(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE))).andThen(frontIntakeFeed(m_launcher, m_intake, m_robotSystem));
  }
    
  
    /** Intake logic to set up the rest of the commands
     * 
     * @param m_launcher Launcher subsystem
     * @param m_intake Intake subsystem
     * @param m_System RobotSystem subsystem
     * @return command group logic that runs the "right" intake
     */
  public static Command intake(Launcher m_launcher, Intake m_intake, RobotSystem m_robotSystem){
    return Commands.run(() -> m_robotSystem.setGoalState(SystemState.INTAKE)).until(() -> (m_intake.getFrontNote() == false || m_intake.getRearNote() == false)).andThen(new ConditionalCommand(frontIntake(m_launcher, m_intake, m_robotSystem), rearIntakeNote(m_launcher, m_intake, m_robotSystem), () -> m_intake.getFrontNote() == false));
  }

}
