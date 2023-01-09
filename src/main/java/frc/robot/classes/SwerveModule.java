// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** This is our custom class that represents an entire swerve module.
 * It contains functions for getting and setting modules, as well as a few other things.
 */
public class SwerveModule {
    
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder azimuthEncoder;

    private double azimuthOffset;
    private double startingAzimuth;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

    SwerveModule (int driveMotorID, int steerMotorID, int azimuthEncoderID, double offset) {
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        azimuthEncoder = new CANCoder(azimuthEncoderID);

        azimuthOffset = offset;
        startingAzimuth = azimuthEncoder.getAbsolutePosition();
    }

    public void setState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = Calculations.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)); 
        lastAngle = angle;
    }

    //#region GET FUNCTIONS
    public SwerveModuleState getState(){
        double velocity = Calculations.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.wheelCircumference, Constants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Calculations.falconToDegrees(steerMotor.getSelectedSensorPosition(), Constants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public Rotation2d getAngle() { 
        // i am not sure if these signs are right.. I though through a few situations in my head and they seem ok, but needs testing
        return Rotation2d.fromDegrees(azimuthEncoder.getPosition() - startingAzimuth + azimuthOffset);
    }
    //#endregion
}