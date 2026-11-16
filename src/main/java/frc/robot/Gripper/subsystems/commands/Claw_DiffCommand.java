package frc.robot.Gripper.subsystems.commands;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.Is_Left_Calibrate;
import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.Is_Right_Calibraete;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper;

public class Claw_DiffCommand extends Command {

  // =========================================================================
  // Subsystem & Target Angles
  // =========================================================================

  private final SubsystemGripper gripperSub;
  // Removed 'final' so Elastic can update these values live
  private double targetPitchRadians;
  private double targetRollRadians;

  // =========================================================================
  // Constructor
  // =========================================================================

  /** Creates a new Claw_DiffCommand. */
  public Claw_DiffCommand(
      SubsystemGripper gripperSub,
      double targetPitchRadians,
      double targetRollRadians) {
    this.gripperSub = gripperSub;
    this.targetPitchRadians = targetPitchRadians;
    this.targetRollRadians = targetRollRadians;
    addRequirements(gripperSub);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Target Pitch", 
        () -> targetPitchRadians, 
        (x) -> targetPitchRadians = x
    );
    builder.addDoubleProperty(
        "Target Roll", 
        () -> targetRollRadians, 
        (x) -> targetRollRadians = x
    );
  }

  // =========================================================================
  // Command Lifecycle
  // =========================================================================

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Calibration checks
    if (Is_Right_Calibraete) {
      gripperSub.Rightcalibrate();
    }

    if (Is_Left_Calibrate) {
      gripperSub.Leftcalibrate();
    }

    // 2. Move Pitch and Roll simultaneously to target angles
    gripperSub.setWristPositionRadians(targetPitchRadians, targetRollRadians);

    // 3. Auto bottle intake
    gripperSub.runAutoBottleIntake();
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
