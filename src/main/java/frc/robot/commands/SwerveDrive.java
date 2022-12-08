// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {

  private SwerveChassis m_chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final IntSupplier dPadSupplier;

  public SwerveDrive(SwerveChassis chassis, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier, IntSupplier dPadSupplier) {
        m_chassis = chassis;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.dPadSupplier = dPadSupplier;

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
      m_chassis.getGyroRot()),
      dPadSupplier.getAsInt()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0), -1);
  }
}
