// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX motor;

  public IntakeSubsystem() {

    motor = new TalonFX(IntakeConstants.kMotorID);

  }

  public void setPower(double duty) {
    motor.set(duty);
  }

  public void stop() {
    motor.set(0);
  }




  @Override
  public void periodic() {


  }
}
