// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class PathFunctions {
    /**
     * Returns a Trajectory object, given a path to the trajectory file
     * @param path trajectory file path, I am pretty sure that you just need to input PathName.path
     * @return Trajectory object
     */
    public static PathPlannerTrajectory createTrajectory (String path)
    {
        return PathPlanner.loadPath(path, Constants.maxVel, Constants.maxAcc);
    }
    
    /**
     * Creates a PPSwerveControllerCommand to follow a trajectory
     * @param trajectory Trajectory object
     * @param poseSupplier A supplier of type Pose2d
     * @param kinematics Kinematics object
     * @param outputStates Consumer of type SwerveModuleState array
     * @param chassis Subsystem requirements, Variable Arguments
     * @return A PPSwerveControllerCommand object
     */
    public static PPSwerveControllerCommand createSwerveController (
        PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, SwerveDriveKinematics kinematics, Consumer<SwerveModuleState[]> outputStates, Subsystem... chassis)
    {
        return new PPSwerveControllerCommand(
            trajectory,
            poseSupplier,
            kinematics,
            new PIDController(Constants.xErrVel, Constants.kI, Constants.kD),
            new PIDController(Constants.yErrVel, Constants.kI, Constants.kD),
            new ProfiledPIDController(1, Constants.kI, Constants.kD,
                new TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc)), //need to revise this later, I am not sure what is velocity, acceleration, etc.
            outputStates,
            chassis);
    }

    /**
     * DEPRECATED, USE createSwerveController
     * @param xErrVel
     * @param yErrVel
     * @return Holonomic Drive Controller object
     */
    public static HolonomicDriveController createHoloController (double xErrVel, double yErrVel)
    {
        return new HolonomicDriveController(
            new PIDController(Constants.xErrVel, Constants.kI, Constants.kD),
            new PIDController(Constants.yErrVel, Constants.kI, Constants.kD),
            new ProfiledPIDController(1, Constants.kI, Constants.kD, //need to revise this later, I am not sure what is velocity, acceleration, etc.
                new TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc)));
    }
}