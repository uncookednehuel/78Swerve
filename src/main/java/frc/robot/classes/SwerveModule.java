// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** This is our custom class that represents an entire swerve module.
 * It contains functions for getting and setting modules, as well as a few other things.
 */
//https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html helpful thing
public class SwerveModule {
    
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder azimuthEncoder;

    private double azimuthOffset;
    private double startingAzimuth;

    private PIDController azimuthController;

    public SwerveModule (int driveMotorID, int steerMotorID, int azimuthEncoderID, double offset) {
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        azimuthEncoder = new CANCoder(azimuthEncoderID);
        azimuthController = new PIDController(Constants.AZIMUTH_MOTOR_KP, Constants.AZIMUTH_MOTOR_KI, Constants.AZIMUTH_MOTOR_KD);
        azimuthOutput =

        azimuthOffset = offset;
        startingAzimuth = azimuthEncoder.getAbsolutePosition();
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
        azimuthController.reset();
        double desiredAngle = desiredState.angle.getDegrees();

        double currentAngle = getAngle().getDegrees();
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, desiredAngle);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, desiredAngle + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            directionMotor.setGain(1.0);
            azimuthController.setSetpoint(currentAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            directionMotor.setGain(-1.0);
            azimuthController.setSetpoint(currentAngle + setpointAngleFlipped);
        }

        steerMotor.set(ControlMode.PercentOutput, azimuthController.calculate(setpointAngleFlipped));
        azimuthController.enable();

        // Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01)) ? lastAzimuth : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        // steerMotor.set(ControlMode.Position, Calculations.degreesToFalcon(angle.getDegrees(), Constants.STEER_GEAR_RATIO));
        // lastAzimuth = angle;
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

    /**
    * Get the closest angle between the given angles.
    */
    private static double closestAngle(double a, double b)
    {
        // get direction
        double dir = (b % 360.0) - (a % 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }
    //#endregion
}