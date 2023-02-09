// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dave_Intake extends SubsystemBase {
  protected final CANSparkMax leftNeo;
  protected final CANSparkMax rightNeo;
  
  protected final DoubleSolenoid solenoid;
  protected final Compressor compressor;

  /** Creates a new IntakeV1_Lentz. */
  public Dave_Intake() {
    leftNeo = new CANSparkMax(6, MotorType.kBrushless);
    rightNeo = new CANSparkMax(5, MotorType.kBrushless);

    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }
 


  public void setSpeed(double speed) {
    leftNeo.set(speed);
    rightNeo.set(speed);
  }

  
  public void setCompressor(boolean isOn){
    if(isOn){
      compressor.enableDigital();
    }
    else{
      compressor.disable();}
    }



  public void setSolenoid(DoubleSolenoid.Value value) {
    solenoid.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }  
}


