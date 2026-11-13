// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectCommand extends Command {

  private final IntakeSubsystem intake;
  private final DeploySubsystem deploy;


  public EjectCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
    this.intake = intake;
    this.deploy = deploy;

    addRequirements(intake, deploy);
    }

  @Override
  public void initialize() {
    // Deploy the mechanism, and the eject the coral
    deploy.deployIntake();
    intake.ejectCoral();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // Retract the mechanism, stop the rollers and the aligner.
    intake.stopRollersAndAligner();
    deploy.retractIntake();
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
