package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private boolean isFrontRunning = false;
    private boolean isRearRunning = false;
    private final SimpleMotorFeedforward intakeFeedforward;
    private final PIDController frontFeedback;
    private final PIDController rearFeedback;

    public Intake(IntakeIO io) {
        this.io = io;
   switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        intakeFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        frontFeedback = new PIDController(0.05, 0.0, 0.0);
        rearFeedback = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        intakeFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        frontFeedback = new PIDController(0.1, 0.0, 0.0);
        rearFeedback = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        intakeFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        frontFeedback = new PIDController(0.0, 0.0, 0.0);
        rearFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }



        io.setBrakeMode(false, false);
    }

    @Override
    public void periodic(){


      io.updateInputs(inputs);
    }

    //     public boolean currentSpike() {
    //         return (frontTalonFX.getOutputCurrent() > 2.0);
    //     }

    //     public Command stopFrontRollers() {
    //     return runOnce(() -> {
    //         IntakeIOTalonFX.frontTalonFX.set();
    //     });
    // }

    // public Command startFrontRollers() {
    //     return runOnce(() -> {
    //         frontTalonFX.set(0.375);
    //     });
    // }

    // public Command reverseFrontRollers() {
    //     return runOnce(() -> {
    //         frontTalonFX.set(-0.375);
    //     });
    // }



    
}
