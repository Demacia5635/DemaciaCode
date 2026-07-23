// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.demacia.utils.controller.CommandController;
import frc.robot.RobotContainer;

public class DriveCommand extends Command {
  private Chassis chassis;
  private CommandController controller;
  private double direction;
  private ChassisSpeeds speeds;
  private static boolean isPrecisionMode;

  /** Creates a new DriveCommand. */
  public DriveCommand(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    isPrecisionMode = false;
    addRequirements(chassis);
  }

  private void driveByJoystick() {
    direction = RobotContainer.isRed ? 1 : -1;
    double joyX = controller.getLeftY() * direction;
    double joyY = controller.getLeftX() * direction;

    // Calculate r]otation from trigger axes
    double rot = controller.getLeftTrigger() - controller.getRightTrigger();

    double velX = Math.pow(joyX, 2) * chassis.getConfig().maxDriveVelocity * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * chassis.getConfig().maxDriveVelocity * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * chassis.getConfig().maxRotationalVelocity * Math.signum(rot);

    if (isPrecisionMode) {
      velX /= 2;
      velY /= 2;
      velRot /= 2;
    }

    speeds = new ChassisSpeeds(velX, velY, -velRot);

    chassis.setVelocities(speeds);
  }

  public static void setPrecisionMode() {
    isPrecisionMode = !isPrecisionMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPrecisionMode = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveByJoystick();
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