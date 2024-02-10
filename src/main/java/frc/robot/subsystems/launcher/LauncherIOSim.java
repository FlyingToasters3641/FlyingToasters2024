package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants;
import static frc.robot.subsystems.launcher.LauncherConstants.reduction;
public class LauncherIOSim implements LauncherIO {

    private static final double LOOP_PERIOD_SECS = 0.02;

    private double flywheelAppliedVolts = 0;
    private double feederAppliedVolts = 0;
    private double wristAppliedVolts = 0;

    private DCMotorSim launcherRollerSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 6.75, 0.025);
    private DCMotorSim launcherPitchSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 6.75, 0.025);

    private FlywheelSim launcherFlywheelSim = new FlywheelSim(DCMotor.getKrakenX60Foc(2), reduction, 0.025);


    private final PIDController flywheelController = new PIDController(FlywheelConstants.gains.kP(), FlywheelConstants.gains.kI(), FlywheelConstants.gains.kD());
    private  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FlywheelConstants.gains.kS(), FlywheelConstants.gains.kV(), FlywheelConstants.gains.kA());

    private Double setpointRPM = null;

    public LauncherIOSim() {

    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        launcherFlywheelSim.update(LOOP_PERIOD_SECS);
        launcherRollerSim.update(LOOP_PERIOD_SECS);
        launcherPitchSim.update(LOOP_PERIOD_SECS);

        if (setpointRPM != null) {
            runVolts(flywheelController.calculate(launcherFlywheelSim.getAngularVelocityRPM(), setpointRPM) + ff.calculate(setpointRPM));
        }

        inputs.topFlywheelAppliedVolts = flywheelAppliedVolts;
        inputs.feederAppliedVolts = feederAppliedVolts;
        inputs.wristAppliedVolts = wristAppliedVolts;
        inputs.launcherAngleRads = launcherRollerSim.getAngularPositionRad();

        inputs.velocityRadPerSec = launcherFlywheelSim.getAngularVelocityRadPerSec();

    }

    @Override
    public void setFlywheelVelocity(double rpm){
        setpointRPM = rpm;
    }


    @Override
    public void runVolts(double volts) {
        flywheelAppliedVolts = MathUtil.clamp(volts, -12, 12);
        launcherFlywheelSim.setInputVoltage(flywheelAppliedVolts);
    }

    @Override
    public void stop(){
        runVolts(0);
    }

    @Override
    public void setFeederVoltage(double speed){
        feederAppliedVolts = MathUtil.clamp(speed, -12, 12);
        
    }

    @Override
    public void setPID(double kP, double kI, double kD){
        flywheelController.setPID(kP, kI, kD);
    }

    @Override
    public void setFF(double ks, double kV, double kA){
        ff = new SimpleMotorFeedforward(ks, kV, kA);
    }

}
