package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class IntakeCommands {

    private IntakeCommands() {}

    public static Command runFrontSpeed(Intake m_intake, DoubleSupplier axis) {
      return Commands.run(() -> {
        m_intake.runFrontRollers(axis);
      });
    }

    public static Command reverseFrontSpeed(Intake m_intake, DoubleSupplier axis) {
      return Commands.run(() -> {
        m_intake.reverseFrontRollers(axis);
      });
    }

    public static Command runRearSpeed(Intake m_intake, double axis) {
      return Commands.run(() -> {
        m_intake.runRearRollers(axis);
      });
    }

    public static Command reverseRearSpeed(Intake m_intake, DoubleSupplier axis) {
      return Commands.run(() -> {
        m_intake.reverseRearRollers(axis);
      });
    }

    public static Command stopFront(Intake m_intake) {
      return Commands.run(() -> {
        m_intake.stopFront();
      });
    }

    public static Command stopRear(Intake m_intake) {
      return Commands.run(() -> {
        m_intake.stopRear();
      });
    }

    public static Command rearIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.REAR_INTAKE);
        }).until(() -> m_launcher.getLauncherNote() == false).andThen(
            new SequentialCommandGroup(
                Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
                Commands.waitSeconds(0.1))).andThen(
                    Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }

    
    public static Command rearOutakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
        return Commands.run(() -> {
            m_System.setGoalState(RobotSystem.SystemState.REAL_REVERSE_INTAKE);
        });
    }
    
    public static Command humanIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
      return Commands.run(() -> {
          m_System.setGoalState(RobotSystem.SystemState.HUMAN_PLAYER);
      }).until(() -> m_launcher.getLauncherNote() == false).andThen(
          new SequentialCommandGroup(
              Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
              Commands.waitSeconds(0.1))).andThen(
                  Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
    }

    public static Command frontIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
      return Commands.run(() -> {
          m_System.setGoalState(RobotSystem.SystemState.FRONT_INTAKE);
      }).until(() -> m_launcher.getLauncherNote() == false).andThen(
          new SequentialCommandGroup(
              Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
              Commands.waitSeconds(0.1)),
                  Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
  }
    
}
