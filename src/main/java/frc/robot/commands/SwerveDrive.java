// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {

  private SwerveChassis m_chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;

  public SwerveDrive(SwerveChassis chassis, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
        m_chassis = chassis;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;

        addRequirements(m_chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_chassis.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
      xSupplier.getAsDouble(), 
      ySupplier.getAsDouble(), 
      rotSupplier.getAsDouble(), 
      m_chassis.getGyroRot()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
