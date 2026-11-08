// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem intake;


  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
    }

  @Override
  public void initialize() {
    // Deploy the mechanism and start the rollers
    intake.deployIntake();
    intake.intakeCoral();
  
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // Retract the mechanism and stop the rollers
    intake.retractIntake();
    intake.stopRollers(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
