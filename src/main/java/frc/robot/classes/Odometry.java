// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class Odometry {

    public Odometry () {

    }

    static public Pose2d getPose(SwerveDriveOdometry odometry, PhotonCamera camera) {
        // PhotonPipelineResult result = camera.getLatestResult();
        // PhotonTrackedTarget bestTarget = result.getBestTarget();
        // Transform3d transform = bestTarget.getBestCameraToTarget();

        return odometry.getPoseMeters();
    }

    static public void updateOdometry(SwerveModulePosition[] positions, Rotation2d gyroRot, SwerveDriveOdometry odometry) {
        odometry.update(gyroRot, positions); //these are supposed to be set to the real read values, not what is being set to the modules
    }

    static public void resetOdometry(Pose2d pose, Rotation2d gyroAngle, SwerveChassis chassis, SwerveDriveOdometry odometry) {
    odometry.resetPosition(gyroAngle, chassis.getPositions(), pose);
  }
}
