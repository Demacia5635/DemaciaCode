// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem intake;
  private final DeploySubsystem deploy;

  public IntakeCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
    this.intake = intake;
    this.deploy = deploy;
    addRequirements(intake, deploy);
  }

  @Override
  public void initialize() {
    // Deploy the mechanism, and intake the coral
    deploy.deployIntake();
    intake.intakeCoral();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // Retract the mechanism, stop the rollers and stop the aligner
    deploy.retractIntake();
    intake.stopRollersAndAligner();
    intake.setHasCoral();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
