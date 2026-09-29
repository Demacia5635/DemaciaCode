// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.sensors.LimitSwitch;

public class sensoSub extends SubsystemBase {
  /** Creates a new sensoSub. */
  frc.demacia.utils.sensors.LimitSwitchConfig limitSwitchConfig = new frc.demacia.utils.sensors.LimitSwitchConfig(0, "limit").withInvert(true);
  LimitSwitch limitSwitch;
  public sensoSub() {
    limitSwitch = new LimitSwitch(limitSwitchConfig);
  }

  public boolean get(){
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
