package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.classes.PathFunctions;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveChassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public final SwerveChassis m_chassis;
  private final XboxController m_driveController;

  public RobotContainer() {
    m_chassis = new SwerveChassis();

    m_driveController = new XboxController(Constants.driverController);

    m_chassis.setDefaultCommand(new SwerveDrive(
      m_chassis,
      () -> -modifyAxis(triggerAdjust(m_driveController.getLeftY())) * Constants.maxSpeed,
      () -> -modifyAxis(triggerAdjust(m_driveController.getLeftX())) * Constants.maxSpeed,
      () -> -modifyAxis(triggerAdjust(m_driveController.getRightX())) * Constants.maxSpeed,
      () -> m_driveController.getPOV()
      ));

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
            new Trigger(m_driveController::getRightBumper).onTrue(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(1, 0))));
            new Trigger(m_driveController::getRightBumper).onFalse(new InstantCommand(() -> m_chassis.setCenter(new Translation2d(0, 0))));
  }

  public Command getAutonomousCommand() {
    PathPlannerTrajectory trajectory1 = PathFunctions.createTrajectory("Test3");
    // error most likely due to trajectory not properly set, or imported from file, maybe need to export from pathplanner application in a different way
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_chassis::getPose,
    null,
    null,
    null,
    null,
    null,
    m_chassis);
    //PathFunctions.createSwerveController(trajectory1, m_chassis::getPose, m_chassis.getKinematics(), m_chassis::setStates, m_chassis);

    return autoBuilder.fullAuto(trajectory1);
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

  public double triggerAdjust(double in) {
    double upAdjust = 0.3;
    double downAdjust = 0.4;
    double triggers = (1 - upAdjust) + (m_driveController.getRightTriggerAxis() * upAdjust) - (m_driveController.getLeftTriggerAxis() * downAdjust); //calculates the trigger adjustment
    return in * triggers;
  }
}