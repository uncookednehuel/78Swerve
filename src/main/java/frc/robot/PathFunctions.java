// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;

/** Add your docs here. */
public final class PathFunctions {
    public Trajectory createTrajectory (String path) {
        return PathPlanner.loadPath(path, Constants.maxVel, Constants.maxAcc);
    }
}
