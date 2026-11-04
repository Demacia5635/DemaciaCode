// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.DemaciaTrajectory;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.Log;
import frc.demacia.wpilibSwerve.command.SwerveCommand;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathCommand extends Command {
  /** Creates a new PathCommand. */
  DemaciaTrajectory trajectory;
  Chassis chassis;
  SwerveCommand swerveCommand;
  public PathCommand(DemaciaTrajectory trajectory, SwerveCommand swerveCommand) {
    this.chassis = Chassis.getInstance();
    this.trajectory = trajectory;
    this.swerveCommand = swerveCommand;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveCommand.odometry.resetPose(Pose2d.kZero);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Log.log("current pose" + Chassis.getInstance().getPose());
    // chassis.setSpeedsRobotRel(ChassisSpeeds.fromRobotRelativeSpeeds(trajectory.calculateSpeeds(chassis.getChassisSpeedsFieldRel(), chassis.getPose()), chassis.gyro.getRotation2d()));
    swerveCommand.setVelRobotRel(ChassisSpeeds.fromRobotRelativeSpeeds(trajectory.calculateSpeeds(chassis.getChassisSpeedsFieldRel(), RobotPose.getInstance().getEstimatedPose()), chassis.gyro.getRotation2d()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory.isFinishedTrajectory; //(RobotPose.getInstance().getEstimatedPose().getTranslation().minus(trajectory.getEndPoint()).getNorm() < 0.08);
  }
}
