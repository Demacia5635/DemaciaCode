// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanismTesting.shooter.subsystems;

import frc.demacia.utils.mechanisms.BaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.robot.mechanismTesting.shooter.shooterConstants;

public class Shooter extends BaseMechanism {
  /** Creates a new Shooter. */
  public Shooter() {
    super(shooterConstants.NAME, 
    new MotorInterface[] {
       
    }, new SensorInterface[]  {

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
