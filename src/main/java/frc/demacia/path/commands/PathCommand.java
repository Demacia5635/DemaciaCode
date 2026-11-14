package frc.demacia.path.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.DemaciaTrajectory;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.wpilibSwerve.command.SwerveCommand;

public class PathCommand extends Command {
  DemaciaTrajectory trajectory;
  Chassis chassis;
  SwerveCommand swerveCommand;

  public PathCommand(DemaciaTrajectory trajectory, SwerveCommand swerveCommand) {
    this.chassis = Chassis.getInstance();
    this.trajectory = trajectory;
    this.swerveCommand = swerveCommand;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    trajectory.startTrajectory();
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = trajectory.calculateSpeeds(chassis.getChassisSpeedsFieldRel(), RobotPose.getInstance().getEstimatedPose());
    swerveCommand.setVelFieldRel(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return trajectory.getIsFinishedTrajectory();
  }
}
