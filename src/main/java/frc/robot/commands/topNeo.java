// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeV1_Lentz;

public class topNeo extends CommandBase {
  IntakeV1_Lentz m_sub;
  double m_speed = 0;

  /** Creates a new topNeo. */
  public topNeo(IntakeV1_Lentz sub, double speed) {
    m_sub = sub;
    m_speed = speed;
    addRequirements(sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_sub.runTopNeo(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sub.runTopNeo(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
