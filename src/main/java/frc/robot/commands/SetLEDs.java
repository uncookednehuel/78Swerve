// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevBlinkin;

public class SetLEDs extends CommandBase {
  private double val;
  private RevBlinkin blinkin;
  public SetLEDs(double val, RevBlinkin blinkin) {
    this.val = val;
    this.blinkin = blinkin;
  }

  @Override
  public void initialize() {
    blinkin.set(val);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
