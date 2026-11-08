// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectCommand extends Command {

  private final IntakeSubsystem intake;


  public EjectCommand(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
    }

  @Override
  public void initialize() {
    intake.deployIntake();
    intake.ejectCoral();
  
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
