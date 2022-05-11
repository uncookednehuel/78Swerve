package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveChassis;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final SwerveChassis m_chassis;

  private final XboxController m_driveController;

  public RobotContainer() {
    configureButtonBindings();
    m_chassis = new SwerveChassis();

    m_driveController = new XboxController(Constants.driverController);

    m_chassis.setDefaultCommand(new SwerveDrive(m_chassis));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
