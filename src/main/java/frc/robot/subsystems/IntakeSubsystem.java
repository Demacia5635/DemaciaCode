// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.Cancoder;

import static frc.robot.constants.IntakeConstants .*;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor deployMotor;
  private final TalonFXMotor alignerMotor;
  private final Cancoder deployCancoder;

  public IntakeSubsystem() {

    rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_CONFIG);

    deployMotor = new TalonFXMotor(DEPLOY_MOTOR_CONFIG);

    alignerMotor = new TalonFXMotor(ALIGNER_MOTOR_CONFIG);

    deployCancoder = new Cancoder(CANCODER_CONFIG);

  }

  // Set power for all motors
  public void setRollersPower(double speed) {
    rollersMotor.set(speed);
  }
  public void setDeployPower(double speed) {
    deployMotor.set(speed);
  }

  public void setAlignerPower(double speed) {
    alignerMotor.set(speed);
  }

  // Cancoder
  public double getDeployPositionRadians() {
    return Math.toRadians(deployCancoder.getPosition().getValueAsDouble());
  }

  public boolean isFullyDeployed() {
    double currentRadians = getDeployPositionRadians();
    return currentRadians >= (TARGET_DEPLOY_RADIANS - TOLORANCE_RADIANS);
   }

  public boolean isFullyRetracted() {
    double currentRadians = getDeployPositionRadians();
    return currentRadians <= (TARGET_RETRACT_RADIANS + TOLORANCE_RADIANS);
   }

  // Intake/Eject the coral
  public void intakeCoral() {
    setRollersPower(INTAKE_SPEED);
  }
  public void ejectCoral() {
    setRollersPower(EJECT_SPEED);
  }

  // Deploy/Retract the mechanism
  public void deployIntake() {
    setDeployPower(DEPLOY_SPEED);

  }
  public void retractIntake() {
    setDeployPower(RETRACT_SPEED);
    stopRollers();
    stopAligner();
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
  public void stopDeploy() {
    setDeployPower(0);
  }
  public void stopAligner() {
    setAlignerPower(0);
  }
  public void stopAll() {
    stopRollers();
    stopDeploy();
    stopAligner();
  }


  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.setSmartDashboardType("IntakeSubsystem");

      builder.addDoubleProperty("Deploy Encoder Position", () -> deployCancoder.getCurrentPosition(), null);
      builder.addDoubleProperty("Deploy Position (Radians)", () -> getDeployPositionRadians() , null);
      builder.addBooleanProperty("Is Fully Deployed", () -> isFullyDeployed(), null);
      builder.addBooleanProperty("Is Fully Retracted", () -> isFullyRetracted(), null);
  }

  @Override
  public void periodic() {}
}
