// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;

import static frc.robot.constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    IDLE, // both motors aren't working
    INTAKING, // rollers and align motors are moving and intaking the coral
    EJECTING, // rollers and align motors are moving and ejecting the coral
    HAS_CORAL; // the coral is secured inside the robot's perimeter
  }

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor alignerMotor;

  private IntakeState currentIntakeState = IntakeState.IDLE; // defualt is idle

  public IntakeSubsystem() {
    rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_CONFIG);
    alignerMotor = new TalonFXMotor(ALIGNER_MOTOR_CONFIG);
  }

  // Set power for all motors
  public void setRollersPower(double power) {
    rollersMotor.setDuty(power);
  }

  public void setAlignerPower(double power) {
    alignerMotor.setDuty(power);
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
    currentIntakeState = IntakeState.INTAKING;
    setRollersPower(INTAKE_SPEED);
    alignCoral();
  }

  public void ejectCoral() {
    currentIntakeState = IntakeState.EJECTING;
    setRollersPower(EJECT_SPEED);
    reverseAligner();
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

  public void stopRollersAndAligner() {
    currentIntakeState = IntakeState.IDLE;
    stopRollers();
    stopAligner();
  }

  public void setHasCoral() {
    currentIntakeState = IntakeState.HAS_CORAL;
  }

  // Get current intake state
  public IntakeState getCurrentIntakeState() {
    return currentIntakeState;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("DeploySubsystem");

    builder.addDoubleProperty("Rollers Velocity", () -> getRollersVelocity(), null);
    builder.addDoubleProperty("Aligner Velocity", () -> getAlignerVelocity(), null);
    builder.addStringProperty("Current Intake State", () -> currentIntakeState.name(), null);
  }

  @Override
  public void periodic() {}
}
