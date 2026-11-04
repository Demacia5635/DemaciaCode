// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  final private double duty;
  final private IntakeSubsystem subsystem;


  public IntakeCommand(double duty, IntakeSubsystem subsystem) {
    this.duty = duty;
    this.subsystem = subsystem;
    addRequirements(this.subsystem);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.setPower(duty); // Set power
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0); // Stop
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
