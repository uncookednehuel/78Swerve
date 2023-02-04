// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPID extends CommandBase {
  private Arm m_arm;
  private double shoulderPosition;
  private double elbowPosition;

  /** Creates a new setArmPID. */
  public SetArmPID(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    m_arm = arm;
    shoulderPosition = arm.shoulderTarget;
    elbowPosition = arm.elbowTarget;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.shoulderPIDcontroller.disableContinuousInput();
    m_arm.shoulderPIDcontroller.setTolerance(5);
    m_arm.elbowPIDcontroller.disableContinuousInput();
    m_arm.elbowPIDcontroller.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = m_arm.shoulderPIDcontroller.calculate(m_arm.getShoulderAbsolutePosition(), m_arm.shoulderTarget);
    m_arm.setShoulderSpeed(shoulderSpeed);
    double elbowSpeed = m_arm.elbowPIDcontroller.calculate(m_arm.getElbowAbsolutePosition(), m_arm.elbowTarget);
    m_arm.setElbowSpeed(elbowSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setShoulderSpeed(0);
    m_arm.setElbowSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
