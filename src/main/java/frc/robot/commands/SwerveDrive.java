// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {

  private SwerveChassis m_chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final IntSupplier dPadSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;

  public SwerveDrive(
    SwerveChassis chassis,
    DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier,
    IntSupplier dPadSupplier,
    DoubleSupplier lTriggerSupplier, DoubleSupplier rTriggerSupplier)
    {
        m_chassis = chassis;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.dPadSupplier = dPadSupplier;
        this.lTriggerSupplier = lTriggerSupplier;
        this.rTriggerSupplier = rTriggerSupplier;

        addRequirements(m_chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double dPadX = (dPadSupplier.getAsInt() == 0 ? 1 : 0) - (dPadSupplier.getAsInt() == 180 ? 1 : 0);
    double dPadY = (dPadSupplier.getAsInt() == 270 ? 1 : 0) - (dPadSupplier.getAsInt() == 90 ? 1 : 0);
    dPadX = triggerAdjust(dPadX * Constants.dPadVel);
    dPadY = triggerAdjust(dPadY * Constants.dPadVel);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      triggerAdjust(xSupplier.getAsDouble()) * Constants.maxSpeed, 
      triggerAdjust(ySupplier.getAsDouble()) * Constants.maxSpeed, 
      triggerAdjust(rotSupplier.getAsDouble()) * Constants.maxSpeed, 
      m_chassis.getGyroRot());
    
    speeds = new ChassisSpeeds(speeds.vxMetersPerSecond + dPadX, speeds.vyMetersPerSecond + dPadY, speeds.omegaRadiansPerSecond);
    
    m_chassis.setSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  public double triggerAdjust(double in) {
    double upAdjust = 0.3;
    double downAdjust = 0.4;
    //Default speed = 1 - upAdjust
    //Full left trigger = 1 - upAdjust - downAdjust
    //Full right trigger = 1
    double triggers = (1 - upAdjust) + (rTriggerSupplier.getAsDouble() * upAdjust) - (lTriggerSupplier.getAsDouble() * downAdjust);
    return in * triggers;
  }
}
