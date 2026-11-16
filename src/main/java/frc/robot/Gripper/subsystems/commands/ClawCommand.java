// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper;

public class ClawCommand extends Command {
  private final SubsystemGripper gripperSub;

  /** Creates a new ClawCommand. */
  public ClawCommand(SubsystemGripper gripperSub) {
    this.gripperSub = gripperSub;
    addRequirements(gripperSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripperSub.runAutoBottleIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the gripper motor when the command finishes or is canceled
    // Replace with your actual stop method name (e.g., gripperSub.stop();)
    gripperSub.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If your subsystem has a method to check if the bottle is caught, return it here:
    // return gripperSub.hasBottle();
    return false;
  }
}
