package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.RobotEnableValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.IntakeStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class IntakeCommands {
  
  private IntakeCommands() {
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

  public static Command rearIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System, IntakeStates m_intakeSys) {
    return Commands.run(() -> {
      m_intakeSys.setIntakeGoalState(IntakeStates.IntakeSystem.INTAKE);
    }).until(() -> m_launcher.getLauncherNote() == false).andThen(
        new SequentialCommandGroup(
            Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
            Commands.waitSeconds(0.1)))
        .andThen(
            Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
  }

  // public static Command humanIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
  //   return Commands.run(() -> {
  //     m_System.setGoalState(RobotSystem.SystemState.HUMAN_PLAYER);
  //   }).until(() -> m_launcher.getNoteFront() == false).andThen(
  //       new SequentialCommandGroup(
  //           Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
  //           Commands.waitSeconds(0.1)),
  //           Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
  // }

  // public static Command frontIntakeNote(Launcher m_launcher, Intake m_intake,
  // RobotSystem m_System) {
  // return Commands.run(() -> {
  // m_System.setGoalState(RobotSystem.SystemState.FRONT_INTAKE);
  // }).until(() -> m_launcher.getNote() == false).andThen(
  // new SequentialCommandGroup(
  // Commands.runOnce(()->m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
  // Commands.waitSeconds(0.1))).andThen(
  // Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE)));
  // }

  // public static Command frontIntakeNote(Launcher m_launcher, Intake m_intake, RobotSystem m_System) {
  //   return Commands.run(() -> {
  //     m_System.setGoalState(RobotSystem.SystemState.FRONT_INTAKE);
  //   }).until(() -> m_launcher.getNoteFront() == false)
  //       .andThen(Commands.run(() -> m_System.setGoalState(RobotSystem.SystemState.LAUNCHER_FEED))
  //           .until(() -> m_intake.getIntakeNote() == false).andThen(
  //               new SequentialCommandGroup(
  //                   Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
  //                   Commands.waitSeconds(0.1)),
  //               Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE))));
  // // }

  // public static Command Intake(Launcher m_launcher, Intake m_intake, RobotSystem m_System, IntakeStates m_intakeSy) {
  //   return Commands.run(() -> {
  //   m_intakeSys.setIntakeGoalState(IntakeStates.IntakeSystem.FRONT_INTAKE);
  //   }).until(() -> m_launcher.getNoteFront() == false)
  //       .andThen(Commands.run(() -> m_System.setGoalState(RobotSystem.SystemState.LAUNCHER_FEED))
  //           .until(() -> m_intake.getIntakeNote() == false).andThen(
  //               new SequentialCommandGroup(
  //                   Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.REVERSE_INTAKE)),
  //                   Commands.waitSeconds(0.1)),
  //               Commands.runOnce(() -> m_System.setGoalState(RobotSystem.SystemState.IDLE))));
  // }
  
}