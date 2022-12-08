// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class Odometry {
    protected SwerveChassis m_chassis;

    protected PhotonCamera camera = new PhotonCamera(Constants.photonCam); 

    public Odometry (SwerveChassis chassis) {
        m_chassis = chassis;
    }

    public Pose2d getPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        Transform3d transform = bestTarget.getBestCameraToTarget();

        

        return m_chassis.m_odometry.getPoseMeters();
    }

    public void updateOdometry(SwerveModuleState[] states) {
        m_chassis.m_odometry.update(m_chassis.getGyroRot(), states); //these are supposed to be set to the real read values, not what is being set to the modules
    }

    public void resetOdometry(Pose2d pose, Rotation2d gyroAngle) {
    m_chassis.zeroGyro();
    m_chassis.m_odometry.resetPosition(pose, gyroAngle);
  }
}
