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
    // Deploy the mechanism, start the rollers to eject the coral (spin them the other way) and reverse the aligner.
    deploy.deployIntake();
    intake.ejectCoral();
    intake.reverseAligner();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // Retract the mechanism, stop the rollers, and stop the aligner.
    intake.stopRollerAndAligner();
    deploy.retractIntake();
    
  }

  @Override
  public boolean isFinished() {
    // Check if the mechanism has retracted and if it has stop the command.
    return false;
  }
}
