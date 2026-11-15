// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.constants.IntakeConstants.IntakeState;

import static frc.robot.constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor alignerMotor;

  private final TalonFXMotor deployMotor;
  private final DigitalInput deployLimitSwitch;

  private IntakeState state;

  private boolean isCalibrated;

  public IntakeSubsystem() {
    rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_CONFIG);
    alignerMotor = new TalonFXMotor(ALIGNER_MOTOR_CONFIG);
    deployMotor = new TalonFXMotor(DEPLOY_MOTOR_CONFIG);
    deployLimitSwitch = new DigitalInput(LIMIT_SWITCH_ID);

    state = IntakeState.IDLE; // default
    addStateChooser();
  }

  public void addStateChooser() {
    SendableChooser<IntakeState> stateChooser = new SendableChooser<>();
    for (IntakeState state : IntakeState.values()) {
      stateChooser.addOption(state.name(), state);
    }
    stateChooser.onChange(newState -> this.state = newState);
    SmartDashboard.putData("State Chooser", stateChooser);
  }

  // Set power for all motors
  public void setRollersDuty(double duty) {
    rollersMotor.setDuty(duty);
  }

  public void setAlignerDuty(double duty) {
    alignerMotor.setDuty(duty);
  }

  public void setDeployDuty(double duty) {
    deployMotor.setDuty(duty);
  }

  public void setDeployPosition(double position) {
    deployMotor.setMotion(position);
  }

  public void setDeployAngle(double deployAngle) {
    double currentAngle = deployMotor.getCurrentAngle();
    if (Math.abs(currentAngle - deployAngle) < ANGLE_TOLORANCE) {
      stopDeploy();
    } else {
      deployAngle = MathUtil.clamp(deployAngle, CLOSED_DEPLOY_ANGLE, OPEN_DEPLOY_ANGLE); // limit the angle to only
                                                                                         // between the closed angle to
                                                                                         // the open angle
      setDeployPosition(deployAngle);
    }

  }

  public void setCalibrated() {
    isCalibrated = true;
  }

  public boolean isCalibrated() {
    return isCalibrated;
  }
  // Getters

  public double getRollersVelocity() {
    return rollersMotor.getCurrentVelocity();
  }

  public double getIntakeDeployAngle() {
    return Math.toDegrees(deployMotor.getCurrentPosition());
  }

  public boolean isDeployedClosed() {
    return !deployLimitSwitch.get(); // if it's not pressed it returns true so just invert it
  }

  // Stop
  public void stopRollers() {
    rollersMotor.stop();
  }

  public void stopAligner() {
    alignerMotor.stop();
  }

  public void stopDeploy() {
    deployMotor.stop();
  }

  public void stopAll() {
    stopDeploy();
    stopRollers();
    stopAligner();
  }

  // Intake state
  public void setState(IntakeState newState) {
    state = newState;
  }

  public IntakeState getState() {
    return state;
  }


  @Override
  public void periodic() {
  }
}
