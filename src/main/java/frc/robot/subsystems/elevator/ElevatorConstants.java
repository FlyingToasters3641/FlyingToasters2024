package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static double minPos = 21.875;
  public static double maxPos = 41.75;
  


    public enum ElevatorPos {

        STORED_POSITION(0, false),
        AMP_POSITION(0, false), // -34
        CHAIN_POSITION(0, true), // -23
        TRAP_POSITION(0, true), // 131
        PLAYER_STATION_POSITION(0, false),
        SPEAKER_POSITION(0, false); // 148
      
        private double elevatorPosition;
        private boolean runLauncher;
      
      
        private ElevatorPos(double elevatorPosition, boolean runLauncher) {
          this.elevatorPosition = elevatorPosition;
      
          this.runLauncher = runLauncher;

        }
      
        public double getElevatorPosition() {
          return elevatorPosition;
        }
      
        public boolean getRunLauncher() {
          return runLauncher;
        }
      

      
      }
}
