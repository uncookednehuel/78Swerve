package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveChassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;

public class RobotContainer {

  private final SwerveChassis m_chassis;
  private final XboxController m_driveController;

  public RobotContainer() {
    configureButtonBindings();
    m_chassis = new SwerveChassis();

    m_driveController = new XboxController(Constants.driverController);

    m_chassis.setDefaultCommand(new SwerveDrive(
      m_chassis,
      () -> -modifyAxis(m_driveController.getLeftY()) * Constants.maxSpeed,
      () -> -modifyAxis(m_driveController.getLeftX()) * Constants.maxSpeed,
      () -> -modifyAxis(m_driveController.getRightX()) * Constants.maxSpeed
      ));

      configureButtonBindings();
  }

  private void configureButtonBindings() {
    // new Button(m_driveController::getBackButton)
            // No requirements because we don't need to interrupt anything
            // .whenPressed(m_chassis::zeroGyro);
  }

  public Command getAutonomousCommand() {
    PathPlannerTrajectory trajectory1 = PathFunctions.createTrajectory("Test1.path");
    PPSwerveControllerCommand command1 = PathFunctions.createSwerveController(trajectory1, m_chassis::getPose, m_chassis.getKinematics(), m_chassis::setStates, m_chassis);
    return command1;
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
    value = Math.copySign(value * 0.3, value);

    return value;
  }

  public Vector2d triggerAdjust(double inX, double inY) {
    double upAdjust = 0.3;
    double downAdjust = 0.4;
    double triggers = (m_driveController.getRightTriggerAxis() * upAdjust) - (m_driveController.getLeftTriggerAxis() * downAdjust); //calculates the trigger adjustment
    return new Vector2d(inX * triggers, inY * triggers); //returns a normalized vector
  }
}