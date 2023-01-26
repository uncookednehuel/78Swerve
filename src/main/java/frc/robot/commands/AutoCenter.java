// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.LimeLight;
import frc.robot.subsystems.SwerveChassis;

public class AutoCenter extends CommandBase {
  PIDController xController;
  PIDController yController;
  // PIDController rotController;

  Pose2d setPoint;

  ChassisSpeeds speeds;

  LimeLight limeLight;
  SwerveChassis chassis;

  /** Creates a new AutoCenter. */
  public AutoCenter(LimeLight limeLight, Pose2d setPoint, SwerveChassis chassis) {
    xController = new PIDController(1, 0, 0);
    yController = new PIDController(1, 0, 0);
    // rotController = new PIDController(0.3, 0, 0);

    this.limeLight = limeLight;
    this.setPoint = setPoint;
    this.chassis = chassis;

    speeds = new ChassisSpeeds();

    addRequirements(this.chassis);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    // rotController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = limeLight.getCamPose();

    speeds.vxMetersPerSecond = xController.calculate(pose.getX(), setPoint.getX());
    speeds.vyMetersPerSecond = yController.calculate(pose.getY() * -1, setPoint.getY());
    speeds.omegaRadiansPerSecond = 0;//rotController.calculate(pose.getRotation().getRadians(), setPoint.getRotation().getRadians());

    SmartDashboard.putNumber("PIDXError", xController.getPositionError());
    SmartDashboard.putNumber("PIDYError", yController.getPositionError());
    // SmartDashboard.putNumber("PIDRotError", rotController.getPositionError());

    chassis.setSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
