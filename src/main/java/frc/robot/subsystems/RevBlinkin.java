package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RevBlinkin extends SubsystemBase {
  private static Spark m_blinkin = null;
  private Dave_Intake daveIntake;
  public RevBlinkin(Dave_Intake daveIntake) {
    m_blinkin = new Spark(9);
    this.daveIntake = daveIntake;
    //solid_orange();
  }

  public void set(double val) {
    new PrintCommand("Setting LEDs to " + val);
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(0.91);
    }
  }
  
 @Override
  public void periodic() {
    SmartDashboard.putBoolean("Running", daveIntake.hasItem());
    if (daveIntake.hasItem() == true) {                                
      m_blinkin.set(0.75);
    } else {

      m_blinkin.set(0.81);
    }
    m_blinkin.set(0.01);
 }

 
 // public void solid_orange() {
 // set(0.65);
 // }
}


