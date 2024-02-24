package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.Timer;


public interface LEDIO {

    public static int ledStatus = 0;
 
    @AutoLog
    public static class LEDIOInputs {
        static int ledStatus = 0;
        public static int ledBlink = 0;
    }
    
    public default void updateInputs(LEDIOInputs inputs) {}

    
    public default void setColor(String color, int index, int count){

    
    }
    public default void ledSwitch(int status, Timer greenTimer){
        
    }

}
