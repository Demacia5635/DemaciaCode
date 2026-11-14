// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrationCommand extends Command {
  public final IntakeSubsystem intake;
  public CalibrationCommand(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setDeployDuty(-0.1); // moves the deploy motor a bit until it hits the limit switch
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      intake.stopDeploy();
      intake.setCalibrated();
    }

  }

  @Override
  public boolean isFinished() {
    return intake.isDeployedClosed();
  }
}
