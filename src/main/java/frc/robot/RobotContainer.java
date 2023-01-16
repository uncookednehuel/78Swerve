package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.classes.Odometry;
import frc.robot.classes.PathFunctions;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public final SwerveChassis m_chassis;
  private final XboxController m_driveController;

  private final HashMap<String, Command> eventMap;
  private final SwerveAutoBuilder autoBuilder;

  public RobotContainer() {
    m_chassis = new SwerveChassis();

    m_driveController = new XboxController(Constants.driverController);

    m_chassis.setDefaultCommand(new SwerveDrive(
      m_chassis,
      () -> -modifyAxis(m_driveController.getLeftY()),
      () -> -modifyAxis(m_driveController.getLeftX()),
      () -> -modifyAxis(m_driveController.getRightX()),
      () -> m_driveController.getPOV(),
      () -> m_driveController.getLeftTriggerAxis(),
      () -> m_driveController.getRightTriggerAxis()
      ));

    eventMap = new HashMap<>();
    eventMap.put("Waypoint1Reached", new PrintCommand("Waypoint 1 reached!"));
    eventMap.put("command1", new PrintCommand("Hello World"));
    
    autoBuilder = new SwerveAutoBuilder(
      m_chassis::getPose,
      m_chassis::resetPose,
      new PIDConstants(5.0, 0.0, 0.0),
      new PIDConstants(0.5, 0.0, 0.0),
      m_chassis::setSpeedsAuto,
      eventMap,
      m_chassis);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
            new Trigger(m_driveController::getYButton).onTrue(new InstantCommand(m_chassis::zeroGyro));//(new InstantCommand(m_chassis::zeroGyro));
            SwerveModuleState[] emptyStates = {
              new SwerveModuleState(0.01, new Rotation2d()),
              new SwerveModuleState(0.01, new Rotation2d()),
              new SwerveModuleState(0.01, new Rotation2d()),
              new SwerveModuleState(0.01, new Rotation2d())
            };
            new Trigger(m_driveController::getXButton).onTrue(new InstantCommand(() -> m_chassis.setStates(emptyStates, true)));
            new Trigger(m_driveController::getBButton).onTrue(new InstantCommand(() -> m_chassis.resetAllToAbsolute()));
            new Trigger(m_driveController::getRightBumper).onTrue(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(1, 0))));
            new Trigger(m_driveController::getRightBumper).onFalse(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0, 0))));
  }

  public Command getAutonomousCommand() {
    PathPlannerTrajectory trajectory1 = PathFunctions.createTrajectory("2MeterStraight");

    return autoBuilder.followPath(trajectory1);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return (value > 0.0 ? value - deadband : value + deadband) / (1.0 - deadband);
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);
    // Square the axis
    value = Math.copySign(value * 0.4, value);
    return value;
  }
}