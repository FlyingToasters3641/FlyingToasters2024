package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.Timer;


public interface LEDIO {

    
 
    @AutoLog
    public static class LEDIOInputs {

    }
    public static final int ledStatus = 0;
    public default void updateInputs(LEDIOInputs inputs) {}

    
    public default void setColor(String color, int index, int count){

    
    }
    public default void ledSwitch(int status, Timer greenTimer){
        
    }

}
