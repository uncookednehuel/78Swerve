// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

/** This is our custom class that represents an entire swerve module.
 * It contains functions for getting and setting modules, as well as a few other things.
 */
public class SwerveModule {
    
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder azimuthEncoder;

    private double azimuthOffset;
    private double startingAzimuth;
    private Rotation2d lastAzimuth;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

    SwerveModule (int driveMotorID, int steerMotorID, int azimuthEncoderID, double offset) {
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        azimuthEncoder = new CANCoder(azimuthEncoderID);

        azimuthOffset = offset;
        startingAzimuth = azimuthEncoder.getAbsolutePosition();
        lastAzimuth = getState().angle;
    }

    private void config() {
        //drive motor
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(InvertType.InvertMotorOutput);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);
        //steer motor
        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        steerMotor.setInverted(InvertType.InvertMotorOutput);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        resetToAbsolute();
        //azimuth encoder
        azimuthEncoder.configFactoryDefault();
        azimuthEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    //#region SET FUNCTIONS
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        // This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not
        desiredState = Calculations.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Calculations.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01)) ? lastAzimuth : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        steerMotor.set(ControlMode.Position, Calculations.degreesToFalcon(angle.getDegrees(), Constants.STEER_GEAR_RATIO));
        lastAzimuth = angle;
    }
    //#endregion

    //#region GET FUNCTIONS
    public SwerveModuleState getState(){
        double velocity = Calculations.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Calculations.falconToDegrees(steerMotor.getSelectedSensorPosition(), Constants.STEER_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public Rotation2d getAngle() { 
        // i am not sure if these signs are right.. I though through a few situations in my head and they seem ok, but needs testing
        return Rotation2d.fromDegrees(azimuthEncoder.getPosition() - startingAzimuth + azimuthOffset);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Calculations.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
    //#endregion
}