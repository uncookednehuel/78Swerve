package frc.robot.classes;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

/** Add your docs here. */
public class PathFunctions {
    /**
     * Returns a Trajectory object, given a path to the trajectory file
     * @param path trajectory file path, I am pretty sure that you just need to input PathName.path
     * @return Trajectory object
     */
    public static PathPlannerTrajectory createTrajectory (String path) {
        return PathPlanner.loadPath(path, Constants.PATH_MAX_VEL, Constants.PATH_MAX_ACC);
    }
    
    /**
     * DEPRECATED PROBABLY
     * Creates a PPSwerveControllerCommand to follow a trajectory
     * @param trajectory Trajectory object
     * @param poseSupplier A supplier of type Pose2d
     * @param kinematics Kinematics object
     * @param outputStates Consumer of type SwerveModuleState array
     * @param chassis Subsystem requirements, Variable Arguments
     * @return A PPSwerveControllerCommand object
     */
    public static PPSwerveControllerCommand createSwerveController (
        PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, SwerveDriveKinematics kinematics, Consumer<ChassisSpeeds> chassisSpeeds, Subsystem... chassis) {

        return new PPSwerveControllerCommand(
            trajectory,poseSupplier,
            new PIDController(Constants.X_ERROR_VEL, Constants.TRAJECTORY_KI, Constants.TRAJECTORY_KD),
            new PIDController(Constants.Y_ERROR_VEL, Constants.TRAJECTORY_KI, Constants.TRAJECTORY_KD),
            new PIDController(1, Constants.TRAJECTORY_KI, Constants.TRAJECTORY_KD),
            chassisSpeeds,
            chassis);
    }

    /**
     * Resets the odometry of the robot to the provided trajectory intial pose
     * @param m_chassis
     * @param trajectory
     */
    public static void resetOdometry(SwerveChassis m_chassis, PathPlannerTrajectory trajectory) {
        Odometry.resetOdometry(trajectory.getInitialHolonomicPose(), trajectory.getInitialHolonomicPose().getRotation(), m_chassis, m_chassis.odometry);
    }
}