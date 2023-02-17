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

  private double speed;
  private double reverseSpeed;
  private static final double threshold = 10;
  private static final double maxTime = 10; // should be lowered
  private static final double revereseTime = 0.8;
 
  public AutoChargeStation(SwerveChassis chassis, double speed, double reverseSpeed) {
    this.chassis = chassis;
    this.speed = speed;
    this.reverseSpeed = reverseSpeed;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialRot = Math.abs(chassis.getGyroRot(1).getDegrees());
    hasRotated = false;
    hasFlattened = false;
    isReversing = false;
    chassis.setSpeeds(new ChassisSpeeds(speed, 0, 0));
  }

  @Override
  public void execute() {
    if (Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot > threshold) {
      hasRotated = true;
    }
    if ((Math.abs(chassis.getGyroRot(1).getDegrees()) - initialRot < threshold) && hasRotated) {
      hasFlattened = true;
    }
    if (hasRotated && hasFlattened && !isReversing) {
      chassis.setSpeeds(new ChassisSpeeds(reverseSpeed, 0, 0));
      isReversing = true;
      startReverseTime = Timer.getFPGATimestamp();
    }
    SmartDashboard.putNumber("GyroPitch", Math.abs(chassis.getGyroRot(1).getDegrees()));
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