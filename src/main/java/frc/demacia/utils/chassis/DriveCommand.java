// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.RobotCommon;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.log.Log;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private Chassis chassis;
  private CommandController controller;
  private ChassisSpeeds speeds;
  private boolean precisionMode;

  // Last driver inputs / commanded values, cached for diagnostics logging.
  private double leftTrigger;
  private double rightTrigger;
  private double rot;
  private double joyX;
  private double joyY;
  private double velX;
  private double velY;
  private double velRot;

  /** Creates a new DriveCommand. */
  public DriveCommand(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    precisionMode = false;
    addRequirements(chassis);
    addDiagLogs();
  }

  @SuppressWarnings("unchecked")
  private void addDiagLogs() {
    Log.putData("diag/driver/left trigger", new Supplier[] { () -> leftTrigger }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/right trigger", new Supplier[] { () -> rightTrigger }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/rot input", new Supplier[] { () -> rot }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/joy x", new Supplier[] { () -> joyX }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/joy y", new Supplier[] { () -> joyY }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/cmd vx field", new Supplier[] { () -> velX }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/cmd vy field", new Supplier[] { () -> velY }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
    Log.putData("diag/driver/cmd omega", new Supplier[] { () -> velRot }, Log.LogLevel.LOG_AND_NT_NOT_IN_COMP, "", true);
  }

  public void invertPrecisionMode() {
    setPrecisionMode(!precisionMode);
  }

  public void setPrecisionMode(boolean precisionMode) {
    this.precisionMode = precisionMode;
  }

  public boolean getPrecisionMode() {
    return precisionMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.isAutonomous()){
      // clear the cached teleop values so the diag/driver logs don't show stale inputs
      leftTrigger = 0;
      rightTrigger = 0;
      rot = 0;
      joyX = 0;
      joyY = 0;
      velX = 0;
      velY = 0;
      velRot = 0;
      return;
    }

    double direction = RobotCommon.getIsRed() ? 1 : -1;
    joyX = controller.getLeftY() * direction;
    joyY = controller.getLeftX() * direction;
    
    // Calculate r]otation from trigger axes
    leftTrigger = controller.getLeftTrigger();
    rightTrigger = controller.getRightTrigger();
    rot = leftTrigger - rightTrigger;
    
    velX = Math.pow(joyX, 2) * chassis.getMaxDriveVelocity() * Math.signum(joyX);
    velY = Math.pow(joyY, 2) * chassis.getMaxDriveVelocity() * Math.signum(joyY);
    velRot = Math.pow(rot, 2) * chassis.getMaxRotationalVelocity() * Math.signum(rot);
    if(precisionMode){
        velX /= 4;
        velY /= 4;
        // velRot /= 4;
    }

    speeds = new ChassisSpeeds(velX, velY, velRot);
    chassis.setSpeedsFieldRel(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
