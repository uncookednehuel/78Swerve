package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.Odometry;
import frc.robot.classes.SwerveModule;

public class SwerveChassis extends SubsystemBase {

  // abreviations: LU (left upper), RU (right upper), LD (left down), RD (rightdown)
  protected SwerveModule moduleLU, moduleRU, moduleLD, moduleRD;

  protected SwerveDriveKinematics kinematics;
  public SwerveDriveOdometry odometry;
  protected Pigeon2 pidgeon;

  // KINEMATICS
  protected Translation2d centerOfRot;

  protected ChassisSpeeds speeds = new ChassisSpeeds();

  public SwerveChassis() {
    // WILL COULD CONSIDER MAKING THIS AN ARRAY FOR SIMPLIFIED CODE
    moduleLU = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    moduleRU = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    moduleLD = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    moduleRD = new SwerveModule(3, Constants.Swerve.Mod3.constants);

    pidgeon = new Pigeon2(Constants.PIGEON_IMU);

    centerOfRot = new Translation2d();

    pidgeon = new Pigeon2(Constants.PIGEON_IMU);

    kinematics = Constants.Swerve.SWERVE_KINEMATICS;

    speeds = new ChassisSpeeds();

    odometry = new SwerveDriveOdometry(kinematics, getGyroRot(), getPositions());

    Timer.delay(1.0);
    resetAllToAbsolute();

    resetPose(new Pose2d());
  }

  @Override
  public void periodic() {
    SwerveModule swerveModules[] = { moduleLU, moduleRU, moduleLD, moduleRD };
    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  public void resetPose(Pose2d pose) {
    Odometry.resetOdometry(pose, getGyroRot(), this, odometry);
  }

  public void resetAllToAbsolute() {
    moduleLU.resetToAbsolute();
    moduleRU.resetToAbsolute();
    moduleLD.resetToAbsolute();
    moduleRD.resetToAbsolute();
  }

  // #region GYRO

  public void zeroGyro() {
    pidgeon.setYaw(0.0);
  }

  public Rotation2d getGyroRot() {
    return Rotation2d.fromDegrees(pidgeon.getYaw());
  }
  // #endregion
  // #region GET FUNCTIONS

  public Pose2d getPose() {
    SmartDashboard.putNumber("OdometryX", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("OdometryRot", odometry.getPoseMeters().getRotation().getDegrees());
    return Odometry.getPose(odometry);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition positions[] = {
        moduleLU.getPosition(),
        moduleRU.getPosition(),
        moduleLD.getPosition(),
        moduleRD.getPosition(),
    };
    return positions;
  }

  // #endregion
  // #region SET FUNCTIONS

  public void speedsToStates(Boolean isOpenLoop) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, centerOfRot);
    setStates(states, isOpenLoop);
  }

  public void setCenter(Translation2d translation) {
    centerOfRot = translation;
  }

  // Need to find a cleaner way to do this, we shouldn't have 2 set speeds
  // functions, either it should always set the
  // states or never set the states after

  /**
   * Sets chassi speeds, with open loop and without calling speedsToStates
   * 
   * @param speeds
   */
  public void 
  setSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.speeds = speeds;
    SmartDashboard.putNumber("ChassisSpeedsX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedsY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedsRot", speeds.omegaRadiansPerSecond);

    // the the teleop periodic in Robot it calls speeds to states
    if(!isOpenLoop){
      speedsToStates(isOpenLoop);
    }
  }

  /** Overload for setSpeeds, sets speeds to 0, 0, 0 */
  public void setSpeeds() {
    setSpeeds(new ChassisSpeeds(0, 0, 0), false);
  }

  /**
   * Sets chassis speeds, but with a closed loop PID
   * 
   * @param speeds
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    setSpeeds(speeds, false);
  }

  /**
   * Sets the state of each module
   * 
   * @param states     Must be exactly of length 4
   * @param isOpenLoop
   */
  public void setStates(SwerveModuleState[] states, Boolean isOpenLoop) {
    if (states.length != 4) {
      throw new IllegalArgumentException("The \"setStates\" input array size should be 4!");
    } else {
      Odometry.updateOdometry(getPositions(), getGyroRot(), odometry);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);
      moduleRU.setDesiredState(states[1], isOpenLoop);
      moduleLU.setDesiredState(states[0], isOpenLoop);
      moduleLD.setDesiredState(states[2], isOpenLoop);
      moduleRD.setDesiredState(states[3], isOpenLoop);
    }
  }
  // #endregion
}