// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  // for TESTING:
  private double targetDeployAngle;
  private double targetRollersDuty;
  private double targetAlignerDuty;

  private final IntakeSubsystem intake;

  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // for TESTING:
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Target Deploy Angle (Degrees)", () -> targetDeployAngle, (angle) -> targetDeployAngle = Math.toRadians(angle));
    builder.addDoubleProperty("Target Rollers Duty", () -> targetRollersDuty, (duty) -> targetRollersDuty = duty);
    builder.addDoubleProperty("Target Aligner Duty", () -> targetAlignerDuty, (duty) -> targetAlignerDuty = duty);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (intake.getState()) {
      case INTAKING, EJECTING, DEPLOYED_IDLE, CLOSED:
        intake.setDeployAngle(intake.getState().deployAngle);
        intake.setRollersDuty(intake.getState().rollersDuty);
        intake.setAlignerDuty(intake.getState().alignerDuty);
        break;

      case TESTING:
        intake.setDeployAngle(targetDeployAngle);
        intake.setRollersDuty(targetRollersDuty);
        intake.setAlignerDuty(targetAlignerDuty);
        break;

      case IDLE:
      intake.stopAll();
      break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
