package frc.robot.commands;
import frc.robot.subsystems.SwerveChassis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoChargeStation extends CommandBase {
  private SwerveChassis chassis;
  private double initialRot;
  private boolean hasRotated;
  private double startTime;

  private static final double threshold = 10;
  private static final double maxTime = 10; // should be lowered
 
  public AutoChargeStation(SwerveChassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialRot = chassis.getGyroRot(1).getDegrees();
    hasRotated = false;
    chassis.setSpeeds(new ChassisSpeeds(0.8, 0, 0));
  }

  @Override
  public void execute() {
    if (chassis.getGyroRot(1).getDegrees() - initialRot > threshold) {
      hasRotated = true;
    }

    SmartDashboard.putNumber("GyroPitch", chassis.getGyroRot(1).getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  @Override
  public boolean isFinished() {
    return ((chassis.getGyroRot(1).getDegrees() - initialRot < threshold) && hasRotated) || (Timer.getFPGATimestamp() - startTime > maxTime);
  }
}