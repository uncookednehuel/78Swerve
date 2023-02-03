// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class RunArmToTarget extends CommandBase {
  private double elbowTarget;
  private double shoulderTarget;
  private Arm m_Arm;
  
  
  /** Creates a new RunArmToTarget. */
  public RunArmToTarget(Subsystem Arm, double elbowTarget, double shoulderTarget) {
    Arm = m_Arm;
    this.elbowTarget = elbowTarget;
    this.shoulderTarget = shoulderTarget;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.elbowGoToPosition(elbowTarget);
    m_Arm.shoulderGoToPosition(shoulderTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
