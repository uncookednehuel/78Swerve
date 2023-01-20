// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class Odometry {

    public Odometry () { }

    /**
     * Returns the robot's odometry pose
     * @param odometry
     * @return
     */
    static public Pose2d getPose(SwerveDriveOdometry odometry) {
        return odometry.getPoseMeters();
    }

    /**
     * Updates the odometry of the robot
     * @param positions
     * @param gyroRot
     * @param odometry
     */
    static public void updateOdometry(SwerveModulePosition[] positions, Rotation2d gyroRot, SwerveDriveOdometry odometry) {
        odometry.update(gyroRot, positions); //these are supposed to be set to the real read values, not what is being set to the modules
        SmartDashboard.putNumber("OdometryX", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("OdometryY", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("OdometryRot", odometry.getPoseMeters().getRotation().getDegrees());
    }

    /**
     * Resets the robot's odometry
     * @param pose
     * @param gyroAngle
     * @param chassis
     * @param odometry
     */
    static public void resetOdometry(Pose2d pose, Rotation2d gyroAngle, SwerveChassis chassis, SwerveDriveOdometry odometry) {
        odometry.resetPosition(gyroAngle, chassis.getPositions(), pose);
    }
}
