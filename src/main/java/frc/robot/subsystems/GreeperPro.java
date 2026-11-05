// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.OpticalSensor;
import frc.robot.constants.ConstantsGripper;

public class GreeperPro extends SubsystemBase {
  /** Creates a new OpticSensorSub. */
  public TalonSRXMotor m_SRXSensorMotor;
  public OpticalSensor m_opticSensor;

  public GreeperPro() {
    m_SRXSensorMotor = new TalonSRXMotor(ConstantsGripper.kSnowBlowerConfig);
    m_opticSensor = new OpticalSensor(ConstantsGripper.kOpticSensorConfig);
  }


  @Override
  public void initSendable(SendableBuilder builder) { 
    builder.addDoubleProperty("OpticSensorState", ()->m_opticSensor.get(), null);
  }

  public double getOpticSensor() {
    return m_opticSensor.get();
  }

  public void setPower(double power) {
    m_SRXSensorMotor.setDuty(power);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
