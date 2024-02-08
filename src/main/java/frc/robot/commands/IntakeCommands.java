package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

    private IntakeCommands() {}

    // public static Command runFrontSpeed(Intake m_intake, DoubleSupplier axis){
    //     return Commands.run(() -> {m_intake.runFrontRollers(axis);});
    // }
    //   public static Command reverseFrontSpeed(Intake m_intake, DoubleSupplier axis){
    //     return Commands.run(() -> {m_intake.reverseFrontRollers(axis);});
    // }
       public static Command runSpeed(Intake m_intake, DoubleSupplier axis){
        return Commands.run(() -> {m_intake.runRearRollers(axis);m_intake.runFrontRollers(axis);});
    }
      public static Command reverseSpeed(Intake m_intake, DoubleSupplier axis){
        return Commands.run(() -> {m_intake.reverseRearRollers(axis); m_intake.reverseFrontRollers(axis);});
    }
    // public static Command stopFront(Intake m_intake){
    //   return Commands.run(() -> {m_intake.stopFront();});
    // }
    public static Command stopIntake(Intake m_intake){
      return Commands.run(() -> {m_intake.stopRear();m_intake.stopFront();});
    }
    
}
