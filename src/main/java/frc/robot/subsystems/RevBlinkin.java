package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.BlockingQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RevBlinkin extends SubsystemBase {
  BlinkinLEDMode ledMode = BlinkinLEDMode.BLUE;
  private static Spark m_blinkin = null;
  private Dave_Intake daveIntake;
  public RevBlinkin(Dave_Intake daveIntake) {
    m_blinkin = new Spark(9);
    this.daveIntake = daveIntake;
    //solid_orange();
  }

public static enum BlinkinLEDMode{
  YELLOW(0.69),
  PURPLE(0.91),
  RED(0.61),
  BLUE(0.87),
  WHITE(0.93);
  private double value;
  private BlinkinLEDMode(double val){
    this.value = val;
  }
  public double getValue() {
    return value;
  }
}

public void ledMode(BlinkinLEDMode ledMode){
  this.ledMode = ledMode;
}
  
public void set(double BlinkinLEDMode) {
      m_blinkin.set(BlinkinLEDMode);
    // new PrintCommand("Setting LEDs to " + val);
    // SmartDashboard.putNumber("LedValue", val);  
  }
  
 @Override
  public void periodic() {
    SmartDashboard.putBoolean("Running", daveIntake.hasItem());
    // BlinkinLEDMode ledMode = BlinkinLEDMode.BLUE;
    switch(ledMode) {
      case YELLOW:
        set(BlinkinLEDMode.YELLOW.getValue());
        break;
      case PURPLE:
        set(BlinkinLEDMode.PURPLE.getValue());
        break;
      case RED:
        set(BlinkinLEDMode.RED.getValue());
        break;
      case BLUE:
        set(BlinkinLEDMode.BLUE.getValue());
        break;
      case WHITE:
        set(BlinkinLEDMode.WHITE.getValue());
        break;
    }
    
    // if (daveIntake.hasItem() == true) {                                
    //   ledMode(BlinkinLEDMode.WHITE);
    // }

    // if (daveIntake.hasItem() == true) {                                
    //   m_blinkin.set(0.75);
    // } 
    // else {
    //   m_blinkin.set(0.81);
    // }
    // this was put in for testing as nothing else was working, but this effectively changed the color to aqua
 }

 
 // public void solid_orange() {
 // set(0.65);
 // }
}


