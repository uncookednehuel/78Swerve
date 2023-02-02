// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ShoulderManualMovement extends CommandBase {
  protected final XboxController armController;
  protected final Arm m_Arm;
  /** Creates a new ShoulderManualMovement. */
  public ShoulderManualMovement(XboxController m_armController, Arm m_Arm) {
    armController = m_armController;
    this.m_Arm = m_Arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setShoulderSpeed(armController.getRightY() * 0.5);
    
    m_Arm.setElbowSpeed(armController.getLeftY() * 0.5);
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
