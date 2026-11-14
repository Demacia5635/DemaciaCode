// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.Cancoder;

import static frc.robot.constants.IntakeConstants.*;

public class DeploySubsystem extends SubsystemBase {

  public enum DeployState {
    RETRACTED, // the mechanism has fully retracted
    RETRACTING, // the mechanism is in the proccess of retracting
    DEPLOYING, // the mechanism is in the proccess of deploying
    DEPLOYED, // the mechanism has fully deployed
    STOPPED // the mechanism has been emergency stopped
  }

  private final TalonFXMotor deployMotor;
  private final Cancoder deployCancoder;

  private DeployState currentDeployState = DeployState.RETRACTED; // defualt state is retracted

  public DeploySubsystem() {
    deployMotor = new TalonFXMotor(DEPLOY_MOTOR_CONFIG);
    deployCancoder = new Cancoder(CANCODER_CONFIG);
  }

  // Set power & position

  public void setDeployPower(double power) {
    deployMotor.setDuty(power);
  }

  public void setDeployPosition(double position) {
    deployMotor.setMotion(position);
  }

  public double getCurrentDeployPosition() {
    return deployMotor.getCurrentPosition();
  }

  // Cancoder
  public double getDeployPositionRadians() {
    return Math.toRadians(deployCancoder.getPosition().getValueAsDouble());
  }

  // Deployed? / Retracted?
  public boolean isFullyDeployed() {
    double currentPosition = getCurrentDeployPosition();
    return Math.abs(currentPosition - DEPLOYED_POSITION) < POSITION_TOLORANCE;
  }

  public boolean isFullyRetracted() {
    double currentPosition = getCurrentDeployPosition();
    return Math.abs(currentPosition - RETRACTED_POSITION) < POSITION_TOLORANCE;
  }

  // Deploy/Retract the mechanism
  public void deployIntake() {
    currentDeployState = DeployState.DEPLOYING;
    setDeployPosition(DEPLOYED_POSITION);
  }

  public void retractIntake() {
    currentDeployState = DeployState.RETRACTING;
    setDeployPosition(RETRACTED_POSITION);
  }

  // Stop
  public void stopDeploy() {
    currentDeployState = DeployState.STOPPED;
    setDeployPower(0);
  }

  // Get current deploy state
  public DeployState getCurrentDeployState() {
    return currentDeployState;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("DeploySubsystem");

    builder.addDoubleProperty("Deploy Encoder Position", () -> deployCancoder.getCurrentPosition(), null);
    builder.addDoubleProperty("Deploy Position (Radians)", () -> getDeployPositionRadians(), null);
    builder.addBooleanProperty("Is Fully Deployed", () -> isFullyDeployed(), null);
    builder.addBooleanProperty("Is Fully Retracted", () -> isFullyRetracted(), null);
    builder.addStringProperty("Current Deploy State", () -> currentDeployState.name(), null);
  }
  

  @Override
  public void periodic() {
    if (currentDeployState == DeployState.DEPLOYING && isFullyDeployed()) {
      currentDeployState = DeployState.DEPLOYED;
    } else if (currentDeployState == DeployState.RETRACTING && isFullyRetracted()) {
      currentDeployState = DeployState.RETRACTED;
    }

  }

}