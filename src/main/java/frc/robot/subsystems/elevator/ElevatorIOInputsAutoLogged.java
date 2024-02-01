package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;

import org.littletonrobotics.junction.LogTable;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable{

    @Override
    public void toLog(LogTable table) {
        table.put("LeaderAppliedVolts", leaderAppliedVolts);
        table.put("FollowerAppliedVolts", followerAppliedVolts);
    }

    @Override
    public void fromLog(LogTable table) {
        leaderAppliedVolts = table.get("LeaderAppliedVolts", leaderAppliedVolts);
        followerAppliedVolts = table.get("FollowerAppliedVolts", followerAppliedVolts);
    }
    
    public ElevatorIOInputsAutoLogged clone() {
        ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
        copy.leaderAppliedVolts = this.leaderAppliedVolts;
        copy.followerAppliedVolts = this.followerAppliedVolts;
        return copy;
    }
}
