// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Arm extends SubsystemBase {

  private DutyCycleEncoder shoulderEncoder;
  private DutyCycleEncoder elbowEncoder;
  private CANSparkMax shoulderNeo;
  private CANSparkMax elbowNeo;
  private SparkMaxPIDController elbowPIDcontroller;
  private SparkMaxPIDController shoulderPIDcontroller;
  private int target;

  /** Creates a new Arm. */
  public Arm() {
    shoulderNeo = new CANSparkMax(Constants.shoulderNeoID, MotorType.kBrushless);
    elbowNeo = new CANSparkMax(Constants.elbowNeoID, MotorType.kBrushless);
    shoulderEncoder = new DutyCycleEncoder(Constants.shoulderEncoderID);
    elbowEncoder = new DutyCycleEncoder(Constants.elbowEncoderID);
    elbowPIDcontroller = elbowNeo.getPIDController();
    shoulderPIDcontroller = shoulderNeo.getPIDController();
    target = 0;
  }

  /**
   * sets the speed of the shoulder neo
   * @param motorPercentage value to set motor to, accepts -1 to 1
   */
public void setShoulderSpeed(double motorPercentage){
  shoulderNeo.set(motorPercentage);
}

/**
 * sets the speed of the elbow neo
 * @param motorPercentage value to set motor to, accepts -1 to 1
 */
public void setElbowSpeed(double motorPercentage){
  elbowNeo.set(motorPercentage);
}

/**
 * method to get absolute position of shoulder
 * @return double containing absolute position of shoulder
 */
public double getShoulderAbsolutePosition(){
  return shoulderEncoder.getAbsolutePosition() * 360;
}

/**
 * method to get absolute position of elbow
 * @return double containing absolute position of elbow
 */
public double getElbowAbsolutePosition(){
  return elbowEncoder.getAbsolutePosition() * 360;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Encoder", getShoulderAbsolutePosition());
    SmartDashboard.putNumber("Elbow Encoder", getElbowAbsolutePosition());

  }

}