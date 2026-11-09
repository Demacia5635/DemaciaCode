// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.Cancoder;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor deployMotor;
  private final TalonFXMotor alignerMotor;
  private final Cancoder deployCancoder;

  public IntakeSubsystem() {

    rollersMotor = new TalonFXMotor(IntakeConstants.ROLLERS_MOTOR_CONFIG);

    deployMotor = new TalonFXMotor(IntakeConstants.DEPLOY_MOTOR_CONFIG);

    alignerMotor = new TalonFXMotor(IntakeConstants.ALIGNER_MOTOR_CONFIG);

    deployCancoder = new Cancoder(IntakeConstants.CANCODER_CONFIG);

  }

  // Set power for all motors
  public void setRollersPower(double duty) {
    rollersMotor.set(duty);
  }
  public void setDeployPower(double duty) {
    deployMotor.set(duty);
  }

  public void setAlignerPower(double duty) {
    alignerMotor.set(duty);
  }

  // Sensor Getters
  public double getDeployPositionRadians() {
    return Math.toRadians(deployCancoder.getPosition().getValueAsDouble());
  }

  public boolean isFullyDeployed() {
    double currentRadians = getDeployPositionRadians();
    return currentRadians >= IntakeConstants.TARGET_DEPLOY_RADIANS;
   }


  // Intake/Eject
  public void intakeCoral() {
    setRollersPower(IntakeConstants.INTAKE_SPEED);
  }
  public void ejectCoral() {
    setRollersPower(IntakeConstants.EJECT_SPEED);
  }
  // Deploy/Retract
  public void deployIntake() {
    setDeployPower(IntakeConstants.DEPLOY_SPEED);

  }
  public void retractIntake() {
    setDeployPower(IntakeConstants.RETRACT_SPEED);
  }

  public void alignCoral() {
    setAlignerPower(IntakeConstants.ALIGNER_SPEED);
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
    setRollersPower(0);
    setDeployPower(0);
    setAlignerPower(0);
  }




  @Override
  public void periodic() {


  }
}
