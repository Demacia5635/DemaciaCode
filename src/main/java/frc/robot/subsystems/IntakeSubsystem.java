// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.Cancoder;

import static frc.robot.constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor alignerMotor;

  public IntakeSubsystem() {

    rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_CONFIG);

    alignerMotor = new TalonFXMotor(ALIGNER_MOTOR_CONFIG);


  }

  // Set power for all motors
  public void setRollersPower(double speed) {
    rollersMotor.set(speed);
  }

  public void setAlignerPower(double speed) {
    alignerMotor.set(speed);
  }

  // Getters

  public double getAlignerVelocity() {
    return alignerMotor.getCurrentVelocity();
  }

  public double getRollersVelocity() {
    return rollersMotor.getCurrentVelocity();
  }

  // Intake/Eject the coral
  public void intakeCoral() {
    setRollersPower(INTAKE_SPEED);
  }

  public void ejectCoral() {
    setRollersPower(EJECT_SPEED);
  }

  // Align/Reverse the aligner
  public void alignCoral() {
    setAlignerPower(ALIGNER_SPEED);
  }

  public void reverseAligner() {
    setAlignerPower(-ALIGNER_SPEED);
  }

  // Stop
  public void stopRollers() {
    setRollersPower(0);
  }

  public void stopAligner() {
    setAlignerPower(0);
  }

  public void stopRollerAndAligner() {
    stopRollers();
    stopAligner();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.setSmartDashboardType("DeploySubsystem");

      builder.addDoubleProperty("Rollers Velocity", () -> getRollersVelocity(), null);
      builder.addDoubleProperty("Aligner Velocity", () -> getAlignerVelocity() , null);
  }

  @Override
  public void periodic() {
  }
}
