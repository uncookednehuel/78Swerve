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
  private boolean hasFlattened;
  private boolean isReversing;
  private double startTime;
  private double startReverseTime;

  private static final double threshold = 10;
  private static final double maxTime = 10; // should be lowered
  private static final double revereseTime = 0.8;
  private static final double speed = 1;
  private static final double reverseSpeed = -0.7;
 
  public AutoChargeStation(SwerveChassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialRot = chassis.getGyroRot(1).getDegrees();
    hasRotated = false;
    hasFlattened = false;
    isReversing = false;
    chassis.setSpeeds(new ChassisSpeeds(speed, 0, 0));
  }

  @Override
  public void execute() {
    if (chassis.getGyroRot(1).getDegrees() - initialRot > threshold) {
      hasRotated = true;
    }
    if ((chassis.getGyroRot(1).getDegrees() - initialRot < threshold) && hasRotated) {
      hasFlattened = true;
    }
    if (hasRotated && hasFlattened && !isReversing) {
      chassis.setSpeeds(new ChassisSpeeds(reverseSpeed, 0, 0));
      isReversing = true;
      startReverseTime = Timer.getFPGATimestamp();
    }
    SmartDashboard.putNumber("GyroPitch", chassis.getGyroRot(1).getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setSpeeds();
  }

  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startReverseTime > revereseTime) && isReversing) || (Timer.getFPGATimestamp() - startTime > maxTime);
  }
}