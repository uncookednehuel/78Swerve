package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveChassis extends SubsystemBase {

  //abreviations: LU (left upper), RU (right upper), LD (left down), RD (right down)
  protected SwerveModule m_moduleLU, m_moduleRU, m_moduleLD, m_moduleRD;
  private final double offsetLU, offsetRU, offsetLD, offsetRD;
  protected PigeonIMU m_pigeon;

  //  KINEMATICS
  private final Translation2d wheelLU, wheelRU, wheelLD, wheelRD;

  protected double maxSpeed;
  protected double maxVoltage;
  protected ChassisSpeeds m_speeds = new ChassisSpeeds();
  protected SwerveDriveKinematics m_kinematics;

  public SwerveChassis() {

    //double radConst = 0.78539816339744830961566084581988; //1 eighth of 2pi
    // offsetLU = 1.536857001862927; //+ (radConst * 3);
    // offsetRU = 1.986505120311905; //+ (radConst * 5);
    // offsetLD = 3.443786868803264; //+ (radConst * 7);
    // offsetRD = 2.782497338525689; //+ (radConst);

    offsetLU = 1.59677812638971;
    offsetRU = 3.520006539201332;
    offsetLD = 2.879473686459834;
    offsetRD = 2.124563391221613;

    // offsetLU = 0;
    // offsetRU = 0;
    // offsetLD = 0;
    // offsetRD = 0;

    m_moduleLU = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveLUD, Constants.driveLUH, Constants.encLU, offsetLU);
    m_moduleRU = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveRUD, Constants.driveRUH, Constants.encRU, offsetRU);
    m_moduleLD = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveLDD, Constants.driveLDH, Constants.encLD, offsetLD);
    m_moduleRD = Mk4SwerveModuleHelper.createFalcon500(Constants.swerveGearRatio, Constants.driveRDD, Constants.driveRDH, Constants.encRD, offsetRD);

    m_pigeon = new PigeonIMU(Constants.pigeonIMU);

    wheelLU = Constants.wheelLU;
    wheelRU = Constants.wheelRU;
    wheelLD = Constants.wheelLD;
    wheelRD = Constants.wheelRD;

    maxSpeed = Constants.maxSpeed;
    maxVoltage = Constants.maxVoltage;
    m_kinematics = new SwerveDriveKinematics(wheelLU, wheelRU, wheelLD, wheelRD);
  }
  
  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);

    setStates(states);

    SmartDashboard.putNumber("LU ENC", m_moduleLU.getSteerAngle());
    SmartDashboard.putNumber("RU ENC", m_moduleRU.getSteerAngle());
    SmartDashboard.putNumber("LD ENC", m_moduleLD.getSteerAngle());
    SmartDashboard.putNumber("RD ENC", m_moduleRD.getSteerAngle());
  }

  public void zeroGyro() {
    m_pigeon.setFusedHeading(0.0);
  }

  public Rotation2d getGyroRot() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  public void setSpeeds (ChassisSpeeds speeds) {
    m_speeds = speeds;
  }

  public void setStates (SwerveModuleState[] states) {
    if (states.length != 4) {
      throw new IllegalArgumentException("Input array size should be 4");
    } else {
      m_moduleLU.set(states[0].speedMetersPerSecond / maxSpeed * maxVoltage, states[0].angle.getRadians());
      m_moduleRU.set(states[1].speedMetersPerSecond / maxSpeed * maxVoltage, states[1].angle.getRadians());
      m_moduleLD.set(states[2].speedMetersPerSecond / maxSpeed * maxVoltage, states[2].angle.getRadians());
      m_moduleRD.set(states[3].speedMetersPerSecond / maxSpeed * maxVoltage, states[3].angle.getRadians());
    }
  }
}
