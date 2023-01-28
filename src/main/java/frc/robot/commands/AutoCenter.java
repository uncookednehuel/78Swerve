// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.classes.LimeLight;
import frc.robot.subsystems.SwerveChassis;

public class AutoCenter extends CommandBase {
  PIDController xController;
  PIDController yController;
  // PIDController rotController;

  Pose2d setPoint;
  Pose2d apriltagPose;

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

    apriltagPose = getClosestTag();

    speeds = new ChassisSpeeds();

    addRequirements(this.chassis);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    // rotController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = chassis.getFusedPose();

    speeds.vxMetersPerSecond = xController.calculate(pose.getX(), apriltagPose.getX() - setPoint.getX());
    speeds.vyMetersPerSecond = yController.calculate(pose.getY() * -1, apriltagPose.getY() - setPoint.getY());
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

  private Pose2d getClosestTag () { 
    AprilTagFieldLayout apriltags;
    Pose2d closestPose = null;
    double closestDistance = 1000.0;
    int closestID = -1;
    try {

      apriltags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

      Pose2d apriltagPoses[] = new Pose2d[8];
      
      for(int i = 1; i < 9; i++) {
        apriltagPoses[i - 1] = fromPose3d(apriltags.getTagPose(i).get());
      } 

      Pose2d robotPose = chassis.getFusedPose();
      int i = 0;
      for (Pose2d pose : apriltagPoses) {
        double distance = distance(pose, robotPose);
        if(distance < closestDistance) {
          closestDistance = distance;
          closestPose = pose;
          closestID = i + 1;
        }
        i++;
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
   
    if (closestPose != null) {
      SmartDashboard.putNumber("closestDistance", closestDistance);
      SmartDashboard.putNumber("closestPoseX", closestPose.getX());
      SmartDashboard.putNumber("closestPoseY", closestPose.getY());
      SmartDashboard.putNumber("closestPoseID", closestID);
      return closestPose;
    } else {
      DriverStation.reportError("Closest Pose was not set, check getClosestTag in AutoCenter", false);
      return new Pose2d();
    }
  }

  private static Pose2d fromPose3d(Pose3d pose) {
    return new Pose2d(pose.getX(), pose.getY(), new Rotation2d((pose.getRotation().getZ())));
  }

  private static double distance(Pose2d a, Pose2d b) {
    return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
  }
}