// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCalibrationCommand extends Command {
  /** Creates a new ClimbCalibrationCommand. */
  private final ClimbSubsystem climbSubsystem;

  public ClimbCalibrationCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.startCalibration();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!climbSubsystem.isLeftCalibrated()) {
      if (climbSubsystem.isLeftClosed()) {
        climbSubsystem.setLeftCalibrated();
      } else {
        climbSubsystem.setLeftDuty(-0.1);
      }
    }

    if (!climbSubsystem.isRightCalibrated()) {
      if (climbSubsystem.isRightClosed()) {
        climbSubsystem.setRightCalibrated();
      } else {
        climbSubsystem.setRightDuty(-0.1);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbSubsystem.isCalibrated();
  }
}
