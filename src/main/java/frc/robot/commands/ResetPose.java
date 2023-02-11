package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveChassis;

// to be removed
public class ResetPose extends CommandBase {
  private Pose2d pose;
  private SwerveChassis chassis;

  public ResetPose(Pose2d pose, SwerveChassis chassis) {
    this.pose = pose;
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetPose(pose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
