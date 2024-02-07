package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    // creates CANdle in the code;
    // COME BACK LATER TO CHANGE THE NUMBER;
    private CANdle candle = new CANdle(30);
    private final Timer ledTimer = new Timer(); // creates timer for coordinating the strips;
    private final Timer greenTimer = new Timer();
    private boolean ledStatusSwitch; // changes LED status
    private int ledBlink; // blinks the LEDs (unused)
    private int ledStatus; // Which colors should be on or off
    private int[] ledIndex = { 8, 68, 128 }; // Shows where colors start, CHANGE LATER
    private boolean ledDisable;
    private String startLedColor;
    private boolean startLed;
    private int startLedCount;

    public LEDSubsystem() {
        // defines starting variables;
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // sets it to rgb;
        candle.configAllSettings(config);
        ledTimer.start();
        ledStatusSwitch = false;
        ledBlink = 1;
        startLed = false;
        ledStatus = 0;
        ledDisable = true;
        ledTimer.start();

    }

    public void periodic() {
        if (RobotState.isDisabled() == true) {
            ledSwitch(1); // checks if the robot is disabled, if disabled it will display a plain color
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
                ledIndex[0] += 2;
                ledIndex[1] += 2;
                ledIndex[2] += 2;
                // Allows for the LED to loop around the strips
                if (startLed == true) {
                    setColor(startLedColor, 8, startLedCount);
                    startLedCount++;
                    if (startLedCount >= 60) {
                        startLedCount = 0;
                        startLed = false;
                    }
                }
                if (startLed == false) {
                    for (int i = 0; i < ledIndex.length; i++) {
                        if (ledIndex[i] >= 116) {
                            if (i % 3 == 0) {
                                startLedColor = "Pink";
                            }
                            if (i % 3 == 1) {
                                startLedColor = "Blue";
                            }
                            if (i % 3 == 2) {
                                startLedColor = "Purple";
                            }
                            startLed = true;
                        }
                    }
                }

                // Sets the position back to the LED start
                for (int i = 0; i < ledIndex.length; i++) {
                    if (ledIndex[i] > 180) {
                        ledIndex[i] = 8;
                    }

                }
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

                    setColor("green", 8, 100);// CHANGE NUMBER LATER
                }
                if (ledStatusSwitch) {
                    setColor("none", 8, 100);// CHANGE NUMBER LATER
                }
        }
    }

    public void setColor(String color, int index, int count) {

        if (color.equals("Pink")) {
            candle.setLEDs(255, 0, 162, 0, index, count);
        } else if (color.equals("Purple")) {
            candle.setLEDs(128, 0, 255, 0, index, count);
        } else if (color.equals("Blue")) {
            candle.setLEDs(3, 140, 252, 0, index, count);
        }

        else if (color.equals("orange")) {
            candle.setLEDs(252, 144, 3, 0, index, count);
        } else if (color.equals("green")) {
            candle.setLEDs(47, 125, 0, 0, index, count);
        } else if (color.equals("none")) {
            candle.setLEDs(0, 0, 0, 0, index, count);
        }

    }

    public void ledSwitch(int status) { // changes LED status
        if (status == 1) { // red and orange mode

        } else if (status == 1) { // blue
            ledStatus = 1;
        } else if (status == 2) { // timed green flash
            ledStatus = 2;
            greenTimer.reset();
            greenTimer.start();
        }

    }

}
