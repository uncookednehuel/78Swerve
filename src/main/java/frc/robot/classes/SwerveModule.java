// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
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

    public Rotation2d getAngle() { 
        // i am not sure if these signs are right.. I though through a few situations in my head and they seem ok, but needs testing
        return Rotation2d.fromDegrees(azimuthEncoder.getPosition() - startingAzimuth + azimuthOffset);
    }
}