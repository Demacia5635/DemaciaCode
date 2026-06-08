// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanismTesting.turret.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.SensorInterface;
import frc.robot.mechanismTesting.turret.TurretConstants;

public class Turret extends StateBaseMechanism {
  /** Creates a new Shooter. */
  public Turret() {
    super(TurretConstants.NAME, 
    new MotorInterface[] {
       new TalonFXMotor(TurretConstants.MOTOR_CONFIG)
    }, new SensorInterface[]  {
        new LimitSwitch(TurretConstants.LIMIT_SWITCH_CONFIG_MIN), 
        new LimitSwitch(TurretConstants.LIMIT_SWITCH_CONFIG_MAX)
    }, 
    TurretConstants.TURRET_STATES.class
    );
    addLimit(TurretConstants.MOTOR_NAME, TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION);
  }

  public boolean isAtMinLimit() {
    return ((LimitSwitch) getSensor(TurretConstants.LIMIT_SWITCH_NAME_MIN)).get();
  }

  public boolean isAtMaxLimit() {
    return ((LimitSwitch) getSensor(TurretConstants.LIMIT_SWITCH_NAME_MAX)).get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
