// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.robot.subsystems.IntakeSubsystem;

public class ControllerTestingCommand extends Command {
  private final IntakeSubsystem intake;
  private final CommandController controller;
  public ControllerTestingCommand(CommandController controller, IntakeSubsystem intake) {
    this.controller = controller;
    this.intake = intake;

    addRequirements(intake);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setDeployDuty(controller.getLeftY()); 
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
