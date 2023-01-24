package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {

  private SwerveChassis m_chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final IntSupplier dPadSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;

  public SwerveDrive(
      SwerveChassis chassis,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier,
      IntSupplier dPadSupplier,
      DoubleSupplier lTriggerSupplier, DoubleSupplier rTriggerSupplier) {
    m_chassis = chassis;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.dPadSupplier = dPadSupplier;
    this.lTriggerSupplier = lTriggerSupplier;
    this.rTriggerSupplier = rTriggerSupplier;

    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {
  }

  // NEEDS TO BE REVISED, SOMETHING AINT RIGHT
  @Override
  public void execute() {
    double dPadX = (dPadSupplier.getAsInt() == 0 ? 1 : 0) - (dPadSupplier.getAsInt() == 180 ? 1 : 0);
    double dPadY = (dPadSupplier.getAsInt() == 270 ? 1 : 0) - (dPadSupplier.getAsInt() == 90 ? 1 : 0);
    dPadX = triggerAdjust(dPadX) * Constants.DPAD_VEL;
    dPadY = triggerAdjust(dPadY) * Constants.DPAD_VEL;

    SmartDashboard.putNumber("JoystickX", triggerAdjust(xSupplier.getAsDouble()));
    SmartDashboard.putNumber("JoystickY", triggerAdjust(ySupplier.getAsDouble()));
    SmartDashboard.putNumber("JoystickRot", triggerAdjust(rotSupplier.getAsDouble()));

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        triggerAdjust(xSupplier.getAsDouble()) * Constants.Swerve.MAX_SPEED,
        triggerAdjust(ySupplier.getAsDouble()) * Constants.Swerve.MAX_SPEED,
        triggerAdjust(rotSupplier.getAsDouble()) * Constants.Swerve.MAX_ANGULAR_VELOCITY,
        m_chassis.getGyroRot());

    speeds = new ChassisSpeeds(speeds.vxMetersPerSecond + dPadX, speeds.vyMetersPerSecond + dPadY,
        speeds.omegaRadiansPerSecond);

    m_chassis.setSpeeds(speeds, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeeds();
  }

  /**
   * Adjusts the speeds of the given input depending on trigger input, with left
   * trigger decreasing speed and RT increasing
   * 
   * @param in
   * @return Adjusted speed
   */
  public double triggerAdjust(double in) {
    double upAdjust = 0.7;
    double downAdjust = 0.25;
    // Default speed = 1 - upAdjust
    // Full left trigger = 1 - upAdjust - downAdjust
    // Full right trigger = 1
    double triggers = (1 - upAdjust) + (rTriggerSupplier.getAsDouble() * upAdjust)
        - (lTriggerSupplier.getAsDouble() * downAdjust);
    return in * triggers;
  }
}
