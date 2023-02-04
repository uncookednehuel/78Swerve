// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LimeLight {
    NetworkTable table;
    DoubleArraySubscriber camPose;
    DoubleArraySubscriber fieldPose;

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        camPose = table.getDoubleArrayTopic("campose").subscribe(new double[] {});
        fieldPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
    }

    public Pose2d getBotPose() {
        double[] array = fieldPose.get();
            SmartDashboard.putNumber("PoseX", array[0]);
            SmartDashboard.putNumber("PoseY", array[1]);
            SmartDashboard.putNumber("PoseZ", array[2]);
            SmartDashboard.putNumber("PoseXRot", array[3]);
            SmartDashboard.putNumber("PoseYRot", array[4]);
            SmartDashboard.putNumber("PoseZRot", array[5]);

            //offsets because most wpilib uses bottom left field as 0, 0 while limlight has 0, 0 at the center
            return new Pose2d(array[0] + 8.5, array[1] + 4.25, Rotation2d.fromDegrees(array[5]));
    }

    public boolean hasApriltag() {
        return tid.getDouble(-1) != -1; 
    }

    public long getBotPoseTimestamp() {
        long timestamp = fieldPose.getAtomic().timestamp; // might have to be switched to server timestamp
        SmartDashboard.putNumber("lastBotPoseTimestamp", timestamp);
        return timestamp;
    }

    public Pose2d getCamPose() {
        double[] array = camPose.get();
        return new Pose2d(array[2], array[0], Rotation2d.fromDegrees(array[4]));
    }
}
