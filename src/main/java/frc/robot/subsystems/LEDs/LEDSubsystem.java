package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDIO.LEDIOInputs;

public class LEDSubsystem extends SubsystemBase {
    private static final LEDIOInputs inputs = null;
    // creates CANdle in the code;
    // COME BACK LATER TO CHANGE THE NUMBER;
    private final Timer ledTimer = new Timer(); // creates timer for coordinating the strips;
    private final Timer greenTimer = new Timer();
    private boolean ledStatusSwitch; // changes LED status
    private int ledBlink; // blinks the LEDs
    private int ledStatus; // Which colors should be on or off
    private int[] ledIndex = { 0, 61, 121 }; // Shows where colors start, CHANGE LATER
    private boolean ledDisable;
    private LEDIO io;
    public LEDSubsystem(LEDIO io) {
        // defines starting variables;

        ledTimer.start();
        ledStatusSwitch = false;
        ledBlink = 1;
        ledStatus = 0;
        ledDisable = true;
        ledTimer.start();


    }
       
    @Override
    public void periodic() {
        if (RobotState.isDisabled() == true) {
            ledSwitch(1); // checks if the robot is disabled, if disabled it will display a plain orange
            ledDisable = true;
        } else if (RobotState.isEnabled() && ledDisable == true) {
            ledSwitch(0);
            ledDisable = false; // moving colors
        }
        switch (ledStatus) {
            case 0:
                // flowing/moving colors
                ledTimer.advanceIfElapsed(.2); {
                // Moves colors by 2 every .2 seconds
                setColor("Pink", ledIndex[0], 60);// pink CHANGE COUNT AND COLORS LATER
                setColor("Purple", ledIndex[1], 60);// purple CHANGE COUNT AND COLORS LATER
                setColor("Blue", ledIndex[2], 60);// blue CHANGE COUNT AND COLORS LATER
                ledIndex[0]++;
                ledIndex[1]++;
                ledIndex[2]++;
            }
                break;
            case 1:
                setColor("Orange", ledIndex[0], 120); // sets a plain orange color CHANGE COUNT AND COLORS LATER
            case 2:
                // Blinks green every .2 seconds
                if (ledTimer.advanceIfElapsed(.2)) {
                    ledBlink++;
                }

                if (ledBlink % 2 == 0) {
                    ledStatusSwitch = false;
                } else {
                    ledStatusSwitch = true;
                }

                if (ledStatusSwitch) {
                    setColor("Green", 8, 120);// CHANGE NUMBER LATER
                }
                if (ledStatusSwitch) {
                    setColor("none", 8, 120);// CHANGE NUMBER LATER
                }

        }
        io.updateInputs(inputs);
        Logger.recordOutput("LEDs/ledStatus", inputs.ledStatus);
        

    }

    
    public void setColor(String color, int index, int count) {

        io.setColor(color, index, count);
    }

    public void ledSwitch(int status) { // changes LED status
      io.ledSwitch(status, greenTimer);

    }

}