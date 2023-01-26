// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class LimeLight extends SubsystemBase{
    NetworkTable table;
    DoubleArraySubscriber camPose;
    DoubleArraySubscriber fieldPose;

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        camPose = table.getDoubleArrayTopic("campose").subscribe(new double[] {});
        fieldPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    //IT SHOULD ONLY BE A SUBSYSTEM TEMPORARILY, FOR DEBUGGING WITH PERIODIC
    @Override
    public void periodic() {
        SmartDashboard.putNumber("PoseX", getCamPose().getX());
        SmartDashboard.putNumber("PoseY", getCamPose().getY());
        SmartDashboard.putNumber("PoseRot", getCamPose().getRotation().getDegrees());
    }

    public Pose2d getBotPose() {
        double [] array = camPose.get();
        return new Pose2d(array[0], array[1], Rotation2d.fromDegrees(array[5]));
    }

    public Pose2d getCamPose() {
        double[] array = camPose.get();
        return new Pose2d(array[2], array[0], Rotation2d.fromDegrees(array[4]));
    }

    public Pose2d getFusedPose(SwerveChassis chassis, LimeLight limelight) {
        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            chassis.getKinematics(),
            chassis.getGyroRot(),
            chassis.getPositions(),
            chassis.getPose());
        poseEstimator.addVisionMeasurement(limelight.getBotPose(), /*check description of this function for ideas on timestamp input*/);
        return poseEstimator.getEstimatedPosition();
    }
}
