// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeV1_Lentz extends SubsystemBase {
  public CANSparkMax topNeoCone;
  public CANSparkMax bottomNeoCube;
  /** Creates a new IntakeV1_Lentz. */
  public IntakeV1_Lentz() {
    topNeoCone = new CANSparkMax(6, MotorType.kBrushless);
    bottomNeoCube = new CANSparkMax(5, MotorType.kBrushless);
  }
   public CommandBase runTopNeo(double speed){

    return this.runOnce(()->topNeoCone.set(speed));
  }

  public CommandBase runBottomNeo(double speed){
    return this.runOnce(()->bottomNeoCube.set(speed));
  }
  
  public void reverseTopNeo(double speed){
    topNeoCone.set(-0.4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
