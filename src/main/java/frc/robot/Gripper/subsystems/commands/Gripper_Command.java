package frc.robot.Gripper.subsystems.commands;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper;

public class Gripper_Command extends Command {

  // =========================================================================
  // Subsystem & Target Angles
  // =========================================================================

  private final SubsystemGripper gripperSub;
  private double targetPitchRadians;
  private double targetRollRadians;

  // =========================================================================
  // Constructor
  // =========================================================================

  public Gripper_Command(
      SubsystemGripper gripperSub,
      double targetPitchRadians,
      double targetRollRadians) {
    this.gripperSub = gripperSub;
    this.targetPitchRadians = targetPitchRadians;
    this.targetRollRadians = targetRollRadians;
    addRequirements(gripperSub);

    SmartDashboard.putData("Gripper Command", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Target Pitch (Deg)",
        () -> Math.toDegrees(targetPitchRadians),
        (deg) -> targetPitchRadians = Math.toRadians(deg)
    );
    builder.addDoubleProperty(
        "Target Roll (Deg)",
        () -> Math.toDegrees(targetRollRadians),
        (deg) -> targetRollRadians = Math.toRadians(deg)
    );
  }

  // =========================================================================
  // Command Lifecycle
  // =========================================================================

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (gripperSub.getState()) {
      case IDLE:
        gripperSub.stopClaw();
        gripperSub.stopWristMotors();
        break;

      case POSE1:
        // Pitch: 45°, Roll: 90°
        targetPitchRadians = POSE1_PITCH_RADIANS;
        targetRollRadians = POSE1_ROLL_RADIANS;
        gripperSub.setWristPositionRadians(targetPitchRadians, targetRollRadians);
        break;

      case POSE2:
        // Pitch: 0°, Roll: 0°
        targetPitchRadians = POSE2_PITCH_RADIANS;
        targetRollRadians = POSE2_ROLL_RADIANS;
        gripperSub.setWristPositionRadians(targetPitchRadians, targetRollRadians);
        break;

      case SECURING:
        gripperSub.runAutoBottleIntake();
        break;

      case MANUAL:
        // Uses the live Target Pitch (Deg) & Target Roll (Deg) values from Elastic
        gripperSub.setWristPositionRadians(targetPitchRadians, targetRollRadians);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    gripperSub.stopClaw();
    gripperSub.stopWristMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
