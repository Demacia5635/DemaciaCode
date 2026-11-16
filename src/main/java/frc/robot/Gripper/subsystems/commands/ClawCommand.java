// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.subsystems.commands;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.Is_Left_Calibrate;
import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.Is_Right_Calibraete;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper;

public class ClawCommand extends Command {

  // =========================================================================
  // Subsystem & Dependencies
  // =========================================================================

  private final SubsystemGripper gripperSub;

  // =========================================================================
  // Constructor
  // =========================================================================

  /** Creates a new ClawCommand. */
  public ClawCommand(SubsystemGripper gripperSub) {
    this.gripperSub = gripperSub;
    addRequirements(gripperSub);
  }

  // =========================================================================
  // Command Lifecycle
  // =========================================================================

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Is_Right_Calibraete) {
      gripperSub.Rightcalibrate();
    }

    if (Is_Left_Calibrate) {
      gripperSub.Leftcalibrate();
    }
    gripperSub.runAutoBottleIntake();
  }
  

  @Override
  public void end(boolean interrupted) {
    // Stop the gripper motor when the command finishes or is canceled
    // Replace with your actual stop method name (e.g., gripperSub.stop();)
    gripperSub.stopClaw();
  }

  @Override
  public boolean isFinished() {
    // If your subsystem has a method to check if the bottle is caught, return it here:
    // return gripperSub.hasBottle();
    return false;
  }
}
