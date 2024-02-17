package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.leds.LEDIO.LEDIOInputs;

public class LEDCANdle implements LEDIO {

    private CANdle candle = new CANdle(30);
    public int ledStatus;

    @Override
    public void updateInputs(LEDIOInputs inputs) {
    }

    @Override
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
    @Override
    public void ledSwitch(int status, Timer greenTimer) { // changes LED status
        if (status == 1) { // red and orange mode

        } else if (status == 1) { // blue
            ledStatus = 1;
        } else if (status == 2) { // timed green flash
            ledStatus = 2;
            greenTimer.reset();
            greenTimer.start();
        }

}}
