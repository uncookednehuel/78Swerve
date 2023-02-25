package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class SwerveDrive extends CommandBase {

  private SwerveChassis chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final IntSupplier dPadSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  public SwerveDrive(
      SwerveChassis chassis,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier,
      IntSupplier dPadSupplier,
      DoubleSupplier lTriggerSupplier, DoubleSupplier rTriggerSupplier) {
    this.chassis = chassis;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.dPadSupplier = dPadSupplier;
    this.lTriggerSupplier = lTriggerSupplier;
    this.rTriggerSupplier = rTriggerSupplier;

    xLimiter = new SlewRateLimiter(11, -11, 0);
    yLimiter = new SlewRateLimiter(11, -11, 0);
    thetaLimiter = new SlewRateLimiter(30, -30, 0);
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
  }

  // NEEDS TO BE REVISED, SOMETHING AINT RIGHT
  @Override
  public void execute() {
    double dPadX = (dPadSupplier.getAsInt() == 0 ? 1 : 0) - (dPadSupplier.getAsInt() == 180 ? 1 : 0);
    double dPadY = (dPadSupplier.getAsInt() == 270 ? 1 : 0) - (dPadSupplier.getAsInt() == 90 ? 1 : 0);
    // int invertRelative = Math.abs(chassis.getFusedPose().getRotation().getDegrees()) > 90 ? -1 : 1;
    int invertRelative = 1;
    dPadX = triggerAdjust(dPadX) * Constants.DPAD_VEL * invertRelative;
    dPadY = triggerAdjust(dPadY) * Constants.DPAD_VEL * invertRelative;

    SmartDashboard.putNumber("JoystickX", triggerAdjust(xSupplier.getAsDouble()));
    SmartDashboard.putNumber("JoystickY", triggerAdjust(ySupplier.getAsDouble()));
    SmartDashboard.putNumber("JoystickRot", triggerAdjust(rotSupplier.getAsDouble()));

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        triggerAdjust(modifyAxis(xSupplier.getAsDouble())) * Constants.Swerve.MAX_SPEED,
        triggerAdjust(modifyAxis(ySupplier.getAsDouble())) * Constants.Swerve.MAX_SPEED,
        triggerAdjust(modifyAxis(rotSupplier.getAsDouble())) * Constants.Swerve.MAX_ANGULAR_VELOCITY,
        chassis.getGyroRot());

    speeds = new ChassisSpeeds(xLimiter.calculate(speeds.vxMetersPerSecond + dPadX),
        yLimiter.calculate(speeds.vyMetersPerSecond + dPadY),
        thetaLimiter.calculate(speeds.omegaRadiansPerSecond));

    chassis.setSpeeds(speeds, true);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  /**
   * Adjusts the speeds of the given input depending on trigger input, with left
   * trigger decreasing speed and RT increasing
   * 
   * @param in
   * @return Adjusted speed
   */
  public double triggerAdjust(double in) {
    double upAdjust = 0.5;
    double downAdjust = 0.25;
    // Default speed = 1 - upAdjust
    // Full left trigger = 1 - upAdjust - downAdjust
    // Full right trigger = 1
    double triggers = (1 - upAdjust) + (modifyAxis(rTriggerSupplier.getAsDouble()) * upAdjust)
        - (modifyAxis(lTriggerSupplier.getAsDouble()) * downAdjust);
    return in * triggers;
  }
    /**
   * Applies a deadband to the given joystick axis value
   * @param value
   * @param deadband
   * @return
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return (value > 0.0 ? value - deadband : value + deadband) / (1.0 - deadband);
    } else {
      return 0.0;
    }
  }

  /**
   * Processes the given joystick axis value, applying deadband and squaring it
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);
    // Square the axis
    // value = Math.copySign(value * value, value);
    return value;
  }
}
