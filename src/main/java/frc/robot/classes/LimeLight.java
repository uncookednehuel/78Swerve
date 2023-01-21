// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LimeLight extends SubsystemBase{
    NetworkTable table;
    DoubleArraySubscriber camTran;

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        camTran = table.getDoubleArrayTopic("campose").subscribe(new double[] {});
    
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    //IT SHOULD ONLY BE A SUBSYSTEM TEMPORARILY, FOR DEBUGGING WITH PERIODIC
    @Override
    public void periodic() {
        SmartDashboard.putNumber("PoseX", getCamTran().getX());
        SmartDashboard.putNumber("PoseY", getCamTran().getY());
        SmartDashboard.putNumber("PoseRot", getCamTran().getRotation().getDegrees());
    }

    public Pose2d getCamTran() {
        double[] array = camTran.get();
        return new Pose2d(array[2], array[0], Rotation2d.fromDegrees(array[4]));
    }
}
