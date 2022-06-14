package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// I will have to take a look at this if we have problems https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99
public class SwerveChassis extends SubsystemBase {

  //abreviations: LU (left upper), RU (right upper), LD (left down), RD (right down)
  protected SwerveModule m_moduleLU, m_moduleRU, m_moduleLD, m_moduleRD;
  private final double offsetLU, offsetRU, offsetLD, offsetRD;
  protected Pigeon2 m_pigeon;

  protected SwerveModuleState[] states;

  //  KINEMATICS
  private final Translation2d wheelLU, wheelRU, wheelLD, wheelRD;

  protected double maxSpeed;
  protected double maxVoltage;
  protected ChassisSpeeds m_speeds;
  protected SwerveDriveKinematics m_kinematics;
  protected SwerveDriveOdometry m_odometry;

  public SwerveChassis() {

    offsetLU = 0;
    offsetRU = 0;
    offsetLD = 0;
    offsetRD = 0;

    m_moduleLU = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveLUD, Constants.driveLUH, Constants.encLU, offsetLU);
    m_moduleRU = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveRUD, Constants.driveRUH, Constants.encRU, offsetRU);
    m_moduleLD = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveLDD, Constants.driveLDH, Constants.encLD, offsetLD);
    m_moduleRD = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveRDD, Constants.driveRDH, Constants.encRD, offsetRD);

    m_pigeon = new Pigeon2(Constants.pigeonIMU);

    wheelLU = Constants.wheelLU;
    wheelRU = Constants.wheelRU;
    wheelLD = Constants.wheelLD;
    wheelRD = Constants.wheelRD;

    maxSpeed = Constants.maxSpeed;
    maxVoltage = Constants.maxVoltage;
    m_kinematics = new SwerveDriveKinematics(wheelLU, wheelRU, wheelLD, wheelRD);

    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRot(), new Pose2d(0, 0, new Rotation2d())); //we can set starting position and heading
  }
  
  @Override
  public void periodic() {
    m_odometry.update(
    Rotation2d.fromDegrees(getGyroRot().getDegrees() * -1),
    getModuleState(m_moduleLU),
    getModuleState(m_moduleRU),
    getModuleState(m_moduleLD),
    getModuleState(m_moduleRD)); //I am not sure if using a function to return new SwerveModuleState from a SwerveModule is a good idea, but it should work

    setStates(states);
  }

  private SwerveModuleState getModuleState (SwerveModule module) {
    return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle())); //after looking at source code, i am pretty sure .getSteerAngle returns radians
  }

  public void zeroGyro() {
    m_pigeon.setYaw(0.0); //maybe I should add a timeout
    m_odometry.resetPosition(m_odometry.getPoseMeters(), new Rotation2d(0));
  }

  /**
   * Gets the pidgeon's fused heading
   * @return Degrees, clockwise is positive
   */
  public Rotation2d getGyroRot() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw()); //i am not sure if pidgeon 2's clockwise is positive or negative
  }

  public void setSpeeds (ChassisSpeeds speeds) {
    m_speeds = speeds;
    states = m_kinematics.toSwerveModuleStates(m_speeds);
  }

  public void setStates (SwerveModuleState[] states) {
    if (states.length != 4) {
      throw new IllegalArgumentException("Input array size should be 4");
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
      m_moduleLU.set(states[0].speedMetersPerSecond / maxSpeed * maxVoltage, states[0].angle.getRadians());
      m_moduleRU.set(states[1].speedMetersPerSecond / maxSpeed * maxVoltage, states[1].angle.getRadians());
      m_moduleLD.set(states[2].speedMetersPerSecond / maxSpeed * maxVoltage, states[2].angle.getRadians());
      m_moduleRD.set(states[3].speedMetersPerSecond / maxSpeed * maxVoltage, states[3].angle.getRadians());
    }

    //we might need to zero the "states" variable after setting the modules, for safety incase it is not updated
  }

  public SwerveDriveKinematics getKinematics () {
    return m_kinematics;
  }

  public Pose2d getPose () {
    return m_odometry.getPoseMeters();
  }
}
