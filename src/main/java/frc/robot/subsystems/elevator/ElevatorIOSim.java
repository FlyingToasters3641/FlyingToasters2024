package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{
    
    
  private static final double LOOP_PERIOD_SECS = 0.02;

  private ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), 1.0, Units.lbsToKilograms(2), Units.inchesToMeters(.1), Units.inchesToMeters(ElevatorConstants.minPos), Units.inchesToMeters(ElevatorConstants.maxPos));

}
