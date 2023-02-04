// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeV1_Lentz extends SubsystemBase {
  private CANSparkMax topNeoCone;
  private CANSparkMax bottomNeoCube;
  /** Creates a new IntakeV1_Lentz. */
  public IntakeV1_Lentz() {
    topNeoCone = new CANSparkMax(Constants.upperManipNeoID, MotorType.kBrushless);
    bottomNeoCube = new CANSparkMax(Constants.lowerManipNeoID, MotorType.kBrushless);
    
  }
   public CommandBase runTopNeocmd(double speed){

    return this.runOnce(()->topNeoCone.set(speed));
  }

  public CommandBase runBottomNeocmd(double speed){
    return this.runOnce(()->bottomNeoCube.set(speed));
  }
  
  public void reverseTopNeo(double speed){
    topNeoCone.set(-0.4);
  }

  public void stopNeo() {
    bottomNeoCube.set(0);
    topNeoCone.set(0);
  }

  public void testNeo() {
    bottomNeoCube.set(0.5);
    topNeoCone.set(0.5);
  }

  public void runTopNeo(double speed) {
    topNeoCone.set(speed);
  }

  public void runBotNeo(double speed) {
    bottomNeoCube.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
