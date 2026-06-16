// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathCommand extends Command {
  /** Creates a new PathCommand. */
  DemaciaTrajectoryGood trajectory;
  Chassis chassis;
  public PathCommand(DemaciaTrajectoryGood trajectory,Chassis chassis) {
    this.chassis = chassis;
    this.trajectory = trajectory;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // LogManager.log(trajectory.calculateSpeeds(chassis.getChassisSpeedsFieldRel(), chassis.getPose()));
    chassis.setVelocities(trajectory.calculateSpeeds(chassis.getChassisSpeedsFieldRel(), chassis.getPose()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    LogManager.log("stop command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (Math.abs(chassis.getPose().getTranslation().minus(trajectory.getEndPoint()).getNorm()) < 0.5);
    return ((chassis.getPose().getX() == trajectory.getEndPoint().getX() + 0.5) || (chassis.getPose().getX() == trajectory.getEndPoint().getX() - 0.5)) || ((chassis.getPose().getY() == trajectory.getEndPoint().getY() + 0.5) || (chassis.getPose().getY() == trajectory.getEndPoint().getY() - 0.5));
  }
}
