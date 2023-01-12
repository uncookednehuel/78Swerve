package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.angleEnableCurrentLimit, 
            Constants.angleContinuousCurrentLimit, 
            Constants.anglePeakCurrentLimit, 
            Constants.anglePeakCurrentDuration);

        // swerveAngleFXConfig.slot0.kP = Constants.angleKP;
        // swerveAngleFXConfig.slot0.kI = Constants.angleKI;
        // swerveAngleFXConfig.slot0.kD = Constants.angleKD;
        // swerveAngleFXConfig.slot0.kF = Constants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.driveEnableCurrentLimit, 
            Constants.driveContinuousCurrentLimit, 
            Constants.drivePeakCurrentLimit, 
            Constants.drivePeakCurrentDuration);

        // swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        // swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        // swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        // swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // swerveCanCoderConfig.sensorDirection = Constants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}