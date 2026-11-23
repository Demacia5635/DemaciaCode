package frc.robot.Gripper.subsystems.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper.CalibrateSide;

public class Calibrate extends Command {

  private final SubsystemGripper gripperSub;

  public Calibrate(SubsystemGripper gripperSub) {
    this.gripperSub = gripperSub;
    addRequirements(gripperSub);
    SmartDashboard.putData("Calibrate Command", this);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (gripperSub.getCalibrateSide()) {
      case LEFT:
        gripperSub.Leftcalibrate();
        break;
      case RIGHT:
        gripperSub.Rightcalibrate();
        break;
      case NONE:
      default:
        gripperSub.stopWristMotors();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    gripperSub.stopWristMotors();
  }

  @Override
  public boolean isFinished() {
    CalibrateSide side = gripperSub.getCalibrateSide();
    if (side == CalibrateSide.LEFT || side == CalibrateSide.RIGHT) {
      return gripperSub.isPitchAtHomeLimit();
    }
    return false;
  }
}
