// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class limelite extends SubsystemBase {
  /** Creates a new limelite. */
  LimitSwitchConfig lilSwitchConfig = new LimitSwitchConfig(0, "danile");
  LimitSwitch limitSwitch = new LimitSwitch(lilSwitchConfig);
  public limelite() {
    
  }

  public boolean get(){
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
