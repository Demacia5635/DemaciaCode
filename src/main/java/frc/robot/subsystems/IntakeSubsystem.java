// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor deployMotor;

  public IntakeSubsystem() {

    rollersMotor = new TalonFXMotor(IntakeConstants.ROLLERS_MOTOR_CONFIG);

    deployMotor = new TalonFXMotor(IntakeConstants.DEPLOY_MOTOR_CONFIG);

  }

  // Set power for both motors
  public void setRollersPower(double duty) {
    rollersMotor.set(duty);
  }

  public void setDeployPower(double duty) {
    deployMotor.set(duty);
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

  // Stop
  public void stopRollers() {
    rollersMotor.set(0);
  } 
  public void stopDeploy() {
    deployMotor.set(0);
  }

  public void stopAll() {
    rollersMotor.set(0);
    deployMotor.set(0);
  }




  @Override
  public void periodic() {


  }
}
