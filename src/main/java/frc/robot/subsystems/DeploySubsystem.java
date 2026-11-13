// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.Cancoder;

import static frc.robot.constants.IntakeConstants .*;

public class DeploySubsystem extends SubsystemBase {

  private final TalonFXMotor deployMotor;
  private final Cancoder deployCancoder;

  public DeploySubsystem() {

    deployMotor = new TalonFXMotor(DEPLOY_MOTOR_CONFIG);

    deployCancoder = new Cancoder(CANCODER_CONFIG);

  }

  // Set power for all motors

  
  public void setDeployPower(double speed) {
    deployMotor.set(speed);
  }

  public void setDeployPosition(double position) {
    deployMotor.setMotion(position);
  }

   public double getCurrentPosition() {
    return deployMotor.getCurrentPosition();
   }


  // Cancoder
  public double getDeployPositionRadians() {
    return Math.toRadians(deployCancoder.getPosition().getValueAsDouble());
  }


  // Deployed? / Retracted?
  public boolean isFullyDeployed() {
    double currentPosition = getCurrentPosition();
    return Math.abs(currentPosition - DEPLOYED_POSITION) < POSITION_TOLORANCE;
   }

  public boolean isFullyRetracted() {
    double currentPosition = getCurrentPosition();
    return Math.abs(currentPosition - RETRACTED_POSITION) < POSITION_TOLORANCE;
   }



  // Deploy/Retract the mechanism
  public void deployIntake() {
    deployMotor.setMotion(DEPLOYED_POSITION);
  }
  public void retractIntake() {
    deployMotor.setMotion(RETRACTED_POSITION);
  }


  // Stop

  public void stopDeploy() {
    setDeployPower(0);
}


  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.setSmartDashboardType("DeploySubsystem");

      builder.addDoubleProperty("Deploy Encoder Position", () -> deployCancoder.getCurrentPosition(), null);
      builder.addDoubleProperty("Deploy Position (Radians)", () -> getDeployPositionRadians() , null);
      builder.addBooleanProperty("Is Fully Deployed", () -> isFullyDeployed(), null);
      builder.addBooleanProperty("Is Fully Retracted", () -> isFullyRetracted(), null);
  }

  @Override
  public void periodic() {}

}