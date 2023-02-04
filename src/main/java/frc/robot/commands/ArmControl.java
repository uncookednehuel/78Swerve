// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmControl extends CommandBase {
  protected final Arm arm;
  private DoubleSupplier elbowJoyVal;
  private DoubleSupplier shoulderJoyVal;
  /** Creates a new ArmControl. */
  public ArmControl(Arm m_arm, DoubleSupplier elbowJoyVal, DoubleSupplier shoulderJoyVal) {
    arm = m_arm;
    this.elbowJoyVal = elbowJoyVal;
    this.shoulderJoyVal = shoulderJoyVal;
    addRequirements(m_arm);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   arm.setShoulderSpeed(shoulderJoyVal.getAsDouble() * 0.5);
   arm.setElbowSpeed(elbowJoyVal.getAsDouble() * 0.5);
   SmartDashboard.putNumber("shoulderJoy", shoulderJoyVal.getAsDouble());
   SmartDashboard.putNumber("elbowJoy", elbowJoyVal.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setShoulderSpeed(0);
    arm.setElbowSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
