// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.OpticalSensor;
import frc.demacia.utils.sensors.UltraSonicSensor;
import frc.robot.constants.ConstantsGripper;

public class GreeperPro extends SubsystemBase {
  /** Creates a new OpticSensorSub. */
  public TalonSRXMotor m_SRXSensorMotor;
  public OpticalSensor m_opticSensor;
  public UltraSonicSensor ultraSonicSensor;

  public GreeperPro() {
    m_SRXSensorMotor = new TalonSRXMotor(ConstantsGripper.kSnowBlowerConfig);
    // m_opticSensor = new OpticalSensor(ConstantsGripper.kOpticSensorConfig);
    ultraSonicSensor = new UltraSonicSensor(ConstantsGripper.kUltraSonicSensorConfig);
    SmartDashboard.putData(new InstantCommand(() -> setPower(-0.6)));
    SmartDashboard.putData(this);
  }


  @Override
  public void initSendable(SendableBuilder builder) { 
    builder.addDoubleProperty("OpticSensorState", ()->ultraSonicSensor.get(), null);
    builder.addDoubleProperty("Current", ()->m_SRXSensorMotor.getCurrentCurrent(), null);
  }

  public double getOpticSensor() {
    return m_opticSensor.get();
  }

  public void setPower(double power) {
    m_SRXSensorMotor.setDuty(power);    
  }
  public double getUltraSonicSensor() {
    return ultraSonicSensor.get();
  }
  public double getCurrent() {
    return m_SRXSensorMotor.getCurrentCurrent();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
