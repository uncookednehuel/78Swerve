// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  private double elbowTarget;
  private double shoulderTarget;
  private Arm arm;
  private TrapezoidProfile elbowProfile;  
  private TrapezoidProfile shoulderProfile;
  private TrapezoidProfile.State elbow_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State elbow_setpoint = new TrapezoidProfile.State();


  /** Creates a new RunArmToTarget. */
  public SetArm(Arm arm, double elbowTarget, double shoulderTarget) {
    this.arm = arm;
    this.elbowTarget = elbowTarget;
    this.shoulderTarget = shoulderTarget;
  }

  @Override
  public void initialize() {
System.out.println("starting trapezoid");
    //  elbowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(10,0.1),
    //    new TrapezoidProfile.State(elbowTarget, 0), 
    //    new TrapezoidProfile.State(arm.getElbowAbsolutePosition(), 0));
    shoulderProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(150,90),
     new TrapezoidProfile.State(shoulderTarget, 0), 
     new TrapezoidProfile.State(arm.getShoulderAbsolutePosition(), 0));
    elbow_setpoint = new TrapezoidProfile.State(arm.getElbowAbsolutePosition(), 0);
  }

  @Override
  public void execute() { 
    elbow_goal = new TrapezoidProfile.State(elbowTarget, 0);
    elbowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(150,90),
    elbow_goal,
    elbow_setpoint);
    elbow_setpoint = elbowProfile.calculate(0.05);
    double shoulder = shoulderProfile.calculate(0.05).position;
    System.out.println(elbow_setpoint.position);
    //System.out.println(shoulder);
    arm.elbowTarget = elbow_setpoint.position;
    arm.shoulderTarget = shoulder;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.print("SetArm command ended!");
    arm.shoulderPIDcontroller.reset();
    arm.elbowPIDcontroller.reset();
  }

  @Override
  public boolean isFinished() {
    //return Math.abs(arm.shoulderPIDcontroller.getPositionError()) < 2 && Math.abs(arm.elbowPIDcontroller.getPositionError()) < 2;
    return false;
  }
}