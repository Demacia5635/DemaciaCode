// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pathCommand extends Command {
  /** Creates a new pathCommand. */
  demaciaTrajectory trajectory;
  Chassis chassis;
  public pathCommand(demaciaTrajectory trajectory, Chassis chassis) {
    this.trajectory = trajectory;
    this.chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chassis.getInstance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setVelocities(trajectory.getChassisSpeeds(chassis.getPose()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (chassis.getPose().getX() >= trajectory.getPointLestPose().getX() - pathConstans.TOLERANCE) && (chassis.getPose().getY() >= trajectory.getPointLestPose().getY() - pathConstans.TOLERANCE);
  }
}
