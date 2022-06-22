// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {
  
  private SwerveChassis m_chassis;
  private XboxController m_control;

  private final double upAdjust = 0.3;
  private final double downAdjust = 0.4;

  public SwerveDrive(SwerveChassis chassis, XboxController controller) {
    m_chassis = chassis;
    m_control = controller;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // Vector2d adjustedSpeeds = triggerAdjust(m_control.getLeftX(), m_control.getLeftY(), upAdjust, downAdjust);
    m_chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
      m_control.getLeftY(),
      m_control.getLeftX(), 
      m_control.getRightX(), 
      m_chassis.getGyroRot()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Vector2d triggerAdjust(double inX, double inY, double upAdjust, double downAdjust) {
    double triggers = (m_control.getRightTriggerAxis() * upAdjust) - (m_control.getLeftTriggerAxis() * downAdjust); //calculates the trigger adjustment
    return new Vector2d(inX * triggers, inY * triggers); //returns a normalized vector
  }
  //overcomplicated version i was making, until I realized that joystick inputs are already normalized because in the hardware they go in a circle
  // double triggers = ((m_control.getRightTriggerAxis() * upAdjust) - (m_control.getLeftTriggerAxis() * downAdjust)); //calculates the trigger adjustment
  //   double magnitude = Math.sqrt(Math.pow(inX / triggers, 2) + Math.pow(inY / triggers, 2)); //calculates the magnitude of the input vector divided the trigger adjustment
  //   return new Vector2d(inX / magnitude, inY / magnitude); //returns a normalized vector
}
