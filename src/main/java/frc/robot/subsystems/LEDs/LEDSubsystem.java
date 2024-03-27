package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RobotSystem.SystemState;

public class LEDSubsystem extends SubsystemBase {
    private final Timer ledTimer = new Timer(); // creates timer for coordinating the strips;
    private final Timer greenTimer = new Timer();
    private boolean ledStatusSwitch; // changes LED status
    private int ledBlink; // blinks the LEDs

    private int[] ledIndex = { 8, 58, 108 }; // Shows where colors start, CHANGE LATER
    private boolean ledDisable;
    private CANdle candle = new CANdle(30);
    private boolean startLed;
    private String startLedColor;
    private int startLedCount;

    public enum LedStates {

        FLOWING_COLORS,
        ORANGE,
        BLINKING_GREEN,
        IDLE

    }

    private LedStates goalLEDSState = LedStates.IDLE;
    private LedStates ledStatus = LedStates.IDLE;

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.GRBW;
        config.brightnessScalar = 1; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        ledStatusSwitch = false;
        ledBlink = 0;
        startLed = false;
        ledDisable = true;

        ledTimer.start();
        greenTimer.start();
    }

    @Override
    public void periodic() {
        switch(goalLEDSState) {
case IDLE  -> ledStatus = LedStates.IDLE;  
case  FLOWING_COLORS  -> ledStatus = LedStates.FLOWING_COLORS;
case ORANGE -> ledStatus = LedStates.ORANGE;
case BLINKING_GREEN -> ledStatus = LedStates.BLINKING_GREEN;
        }

        switch (ledStatus) {
            case FLOWING_COLORS -> {
                // flowing/moving colors
                ledTimer.advanceIfElapsed(.2); {
                // Moves colors by 2 every .2 seconds
                setColor("Pink", ledIndex[0], 50);// pink CHANGE COUNT AND COLORS LATER
                setColor("Purple", ledIndex[1], 50);// purple CHANGE COUNT AND COLORS LATER
                setColor("Blue", ledIndex[2], 50);// blue CHANGE COUNT AND COLORS LATER
                ledIndex[0]++;
                ledIndex[1]++;
                ledIndex[2]++;

                // loops the leds
                if (startLed == true) {
                    setColor(startLedColor, 8, startLedCount);
                    startLedCount++;
                    if (startLedCount >= 50) {
                        startLedCount = 0;
                        startLed = false;
                    }
                }
                if (startLed == false) {
                    for (int i = 0; i < 3; i++) {
                        if (i == 1) {
                            startLedColor = "Pink";
                        }
                        if (i ==  2  ){
                            startLedColor = "Purple";

                        }
                        if (i == 3) {
                            startLedColor = "Blue";
                        }
                        startLed = true;
                    }
                }
            }
                // sets it back to the start
                for (int i = 0; i < 3; i++) {
                    if (ledIndex[i] > 150) {
                        ledIndex[i] = 8;
                    }
                }

                }
            case ORANGE ->{
                setColor("Orange", 8, 150);
                // sets a plain orange color CHANGE COUNT AND COLORS LATER
            }
            case BLINKING_GREEN ->{
                // Blinks green every .2 seconds
                if (ledTimer.advanceIfElapsed(.2)) {
                    ledBlink++;

                    if (ledBlink % 2 == 0) {
                        setColor("Green", 8, 150);// CHANGE NUMBER LATER
                    } else {
                        setColor("none", 8, 150);// CHANGE NUMBER LATER
                    }
                    if (greenTimer.advanceIfElapsed(.2)) {
                        setLEDSGoalState(LedStates.FLOWING_COLORS);
                    }

                }}
        }
    }

  


    public void setLEDSGoalState(LedStates m_ledsgoalState){
     goalLEDSState = m_ledsgoalState;
    }

    public void setColor(String color, int index, int count) {
        if (color.equals("Pink")) {
            candle.setLEDs(255, 0, 162, 0, index, count);
        } else if (color.equals("Purple")) {
            candle.setLEDs(128, 0, 255, 0, index, count);
        } else if (color.equals("Blue")) {
            candle.setLEDs(3, 140, 252, 0, index, count);
        } else if (color.equals("Orange")) {
            candle.setLEDs(252, 119, 3, 0, index, count);
        } else if (color.equals("Green")) {
            candle.setLEDs(47, 125, 0, 0, index, count);
        } else if (color.equals("none")) {
            candle.setLEDs(0, 0, 0, 0, index, count);
        }
    }
    
    

}
