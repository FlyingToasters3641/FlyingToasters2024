
package frc.robot.subsystems.LEDs;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.Timer;

public class LEDCANdle implements LEDIO {

    private CANdle candle = new CANdle(30);
    public int ledStatus;
    public int ledBlink;

    
    public LEDCANdle () {    
        
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRBW; // set the strip type to RGB
    config.brightnessScalar = 1; // dim the LEDs to half brightness
    candle.configAllSettings(config);
 }
    

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.ledStatus = ledStatus;
        inputs.ledBlink = ledBlink;
    }

    @Override
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

    @Override
    public void ledSwitch(int status, Timer greenTimer) { // changes LED status
        if (status == 1) { // ORANGE
            ledStatus = 1;
        } else if (status == 2) { // timed green flash
            ledStatus = 2;
            greenTimer.reset();
            greenTimer.start();
        }

    }
}
