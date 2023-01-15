package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.Odometry;
import frc.robot.classes.SwerveModule;

public class SwerveChassis extends SubsystemBase {

  //abreviations: LU (left upper), RU (right upper), LD (left down), RD (right down)
  protected SwerveModule m_moduleLU, m_moduleRU, m_moduleLD, m_moduleRD;

  protected SwerveDriveKinematics m_kinematics;
  public SwerveDriveOdometry m_odometry;
  protected PhotonCamera m_photonCam;
  protected Pigeon2 m_pigeon;

  //  KINEMATICS
  private final Translation2d wheelLU, wheelRU, wheelLD, wheelRD;
  protected Translation2d centerOfRot;

  protected ChassisSpeeds m_speeds = new ChassisSpeeds();

  public SwerveChassis() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    // WILL HAVE TO CONSIDER MAKING THIS AN ARRAY FOR SIMPLIFIED CODE
    m_moduleLU = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    m_moduleRU = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    m_moduleLD = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    m_moduleRD = new SwerveModule(3, Constants.Swerve.Mod3.constants);

    m_pigeon = new Pigeon2(Constants.pigeonIMU);

    wheelLU = Constants.wheelLU;
    wheelRU = Constants.wheelRU;
    wheelLD = Constants.wheelLD;
    wheelRD = Constants.wheelRD;

    m_kinematics = new SwerveDriveKinematics(wheelLU, wheelRU, wheelLD, wheelRD);
    centerOfRot = new Translation2d();

    m_pigeon = new Pigeon2(Constants.pigeonIMU);

    m_photonCam = new PhotonCamera(Constants.photonCam);

    m_kinematics = Constants.Swerve.swerveKinematics;

    m_speeds = new ChassisSpeeds();

    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRot(), getPositions());//new SwerveDriveOdometry(m_kinematics, getGyroRot(), new Pose2d(0, 0, new Rotation2d())); //we can set starting position and heading
  }
  
  @Override
  public void periodic() {
    SwerveModule swerveModules[] = {m_moduleLU, m_moduleRU, m_moduleLD, m_moduleRD};
    for(SwerveModule mod : swerveModules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
  }

  public Pose2d getPose () {
    SmartDashboard.putNumber("OdometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("OdometryRot", m_odometry.getPoseMeters().getRotation().getDegrees());
    return Odometry.getPose(m_odometry, m_photonCam);
  }

  //#region  GYRO

  public void zeroGyro() {
    m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroRot() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }
  //#endregion
  //#region GET FUNCTIONS

  public SwerveDriveKinematics getKinematics () {
    return m_kinematics;
  }

  public SwerveModulePosition[] getPositions () {
    SwerveModulePosition positions[] = {
      m_moduleLU.getPosition(),
      m_moduleRU.getPosition(),
      m_moduleLD.getPosition(),
      m_moduleRD.getPosition(),
    };
    return positions;
  }

  //#endregion
  //#region SET FUNCTIONS

  public void speedsToStates(Boolean isOpenLoop) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds, centerOfRot);
    setStates(states, isOpenLoop);
  }

  public void setCenter(Translation2d translation) {
    centerOfRot = translation;
  }

  public void setSpeeds (ChassisSpeeds speeds, int dPad) {
    // double dPadX = (90 - dPad < 45 ? 1 : 0) - (270 - dPad < 45 ? 1 : 0);
    // double dPadY = (0 - dPad < 45 ? 1 : 0) - (180 - dPad < 45 ? 1 : 0);
    double dPadX = (dPad == 0 ? 1 : 0) - (dPad == 180 ? 1 : 0);
    double dPadY = (dPad == 270 ? 1 : 0) - (dPad == 90 ? 1 : 0);
    double dPadVel = 1;
    m_speeds = new ChassisSpeeds(speeds.vxMetersPerSecond + dPadX * dPadVel,
                                  speeds.vyMetersPerSecond + dPadY * dPadVel,
                                  speeds.omegaRadiansPerSecond);
  }

  public void setStates (SwerveModuleState[] states, Boolean isOpenLoop) {
    if (states.length != 4) {
      throw new IllegalArgumentException("The \"setStates\" input array size should be 4!");
    } else {  
      Odometry.updateOdometry(getPositions(), getGyroRot(), m_odometry);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);
      // SmartDashboard.putNumber("LU voltage", states[0].speedMetersPerSecond / Constants.maxSpeed * Constants.maxVoltage);
      m_moduleRU.setDesiredState(states[1], isOpenLoop);
      m_moduleLU.setDesiredState(states[0], isOpenLoop);
      m_moduleLD.setDesiredState(states[2], isOpenLoop);
      m_moduleRD.setDesiredState(states[3], isOpenLoop);
    }
  }
  //#endregion
}