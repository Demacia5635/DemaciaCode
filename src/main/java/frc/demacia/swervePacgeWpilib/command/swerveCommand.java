// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.swervePacgeWpilib.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.swervePacgeWpilib.wpilibPoseEstimator.wpilibPoseEstimator;
import frc.demacia.swervePacgeWpilib.wpilib_kinmatics.wpilibKinmatics;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.swervePacgeWpilib.wpilib_odmetry.wpilivOdmetry;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class swerveCommand extends Command {
  /** Creates a new swerveCommand. */

  Chassis chassis = Chassis.getInstance();
  wpilibKinmatics kinmatics;
  wpilivOdmetry odmetry;
  wpilibPoseEstimator poseEstimator;
  DriveCommand driveCommand;
  SwerveModulePosition[] modulePositions= new SwerveModulePosition[4];
  public swerveCommand(DriveCommand driveCommand, Pose2d visionPose) {
    this.driveCommand = driveCommand;
    for (int i = 0; i < modulePositions.length; i++) modulePositions[i] = chassis.modules[i].getModulePosition();
    kinmatics = new wpilibKinmatics(chassis.modulePositions);
    odmetry = new wpilivOdmetry(kinmatics.kinematics(), chassis.getGyroAngle(), modulePositions);
    poseEstimator = new wpilibPoseEstimator(kinmatics.kinematics(), odmetry.getOdometry(), null, null);
    poseEstimator.updateVision(visionPose, 0.02);
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

