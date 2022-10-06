package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveChassis extends SubsystemBase {

  //abreviations: LU (left upper), RU (right upper), LD (left down), RD (right down)
  protected SwerveModule m_moduleLU, m_moduleRU, m_moduleLD, m_moduleRD;
  protected Pigeon2 m_pigeon;

  //  KINEMATICS
  private final Translation2d wheelLU, wheelRU, wheelLD, wheelRD;

  protected ChassisSpeeds m_speeds = new ChassisSpeeds();
  protected SwerveDriveKinematics m_kinematics;

  public SwerveChassis() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    m_moduleLU = Mk4SwerveModuleHelper.createFalcon500(
      tab.getLayout("Left Up Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
    Constants.swerveGearRatio, Constants.driveLUD, Constants.driveLUH, Constants.encLU, Constants.offsetLU);

    m_moduleRU = Mk4SwerveModuleHelper.createFalcon500(
      tab.getLayout("Right Up Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
    Constants.swerveGearRatio, Constants.driveRUD, Constants.driveRUH, Constants.encRU, Constants.offsetRU);

    m_moduleLD = Mk4SwerveModuleHelper.createFalcon500(
      tab.getLayout("left Down Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
      Constants.swerveGearRatio, Constants.driveLDD, Constants.driveLDH, Constants.encLD, Constants.offsetLD);

    m_moduleRD = Mk4SwerveModuleHelper.createFalcon500(
      tab.getLayout("Right Down Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
      Constants.swerveGearRatio, Constants.driveRDD, Constants.driveRDH, Constants.encRD, Constants.offsetRD);

    m_pigeon = new Pigeon2(Constants.pigeonIMU);

    wheelLU = Constants.wheelLU;
    wheelRU = Constants.wheelRU;
    wheelLD = Constants.wheelLD;
    wheelRD = Constants.wheelRD;

    m_kinematics = new SwerveDriveKinematics(wheelLU, wheelRU, wheelLD, wheelRD);
  }
  
  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

    setStates(states);
  }

  public void zeroGyro() {
    m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroRot() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public void setSpeeds (ChassisSpeeds speeds) {
    m_speeds = speeds;
  }

  public void setStates (SwerveModuleState[] states) {
    if (states.length != 4) {
      throw new IllegalArgumentException("The \"setStates\" input array size should be 4!");
    } else {
      m_moduleLU.set(states[0].speedMetersPerSecond / Constants.maxSpeed * Constants.maxVoltage, states[0].angle.getRadians());
      m_moduleRU.set(states[1].speedMetersPerSecond / Constants.maxSpeed * Constants.maxVoltage, states[1].angle.getRadians());
      m_moduleLD.set(states[2].speedMetersPerSecond / Constants.maxSpeed * Constants.maxVoltage, states[2].angle.getRadians());
      m_moduleRD.set(states[3].speedMetersPerSecond / Constants.maxSpeed * Constants.maxVoltage, states[3].angle.getRadians());
    }
  }
}
