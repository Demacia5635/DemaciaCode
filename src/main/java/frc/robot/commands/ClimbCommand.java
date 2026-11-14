// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ClimbConstants.ClimbStage;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {


  private final ClimbSubsystem climbSubsystem;
  private final Chassis chassis;
  private final Timer driveTimer = new Timer();
  private ClimbStage stage;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(ClimbSubsystem climbSubsystem, Chassis chassis) {
    this.climbSubsystem = climbSubsystem;
    this.chassis = chassis;
    addRequirements(climbSubsystem, chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTimer.stop();
    driveTimer.reset();
    chassis.stop();

    stage = ClimbStage.OPENING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
      case OPENING:
        climbSubsystem.setPosition(ClimbConstants.OPEN_POSITION, ClimbConstants.OPEN_POSITION);
        if (climbSubsystem.isAtTarget(ClimbConstants.OPEN_POSITION)) {
          climbSubsystem.stop();
          driveTimer.restart();
          stage = ClimbStage.DRIVING;
        }
        break;

      case DRIVING:
        if (driveTimer.hasElapsed(ClimbConstants.DRIVE_TIME)) {
          driveTimer.stop();
          chassis.stop();
          stage = ClimbStage.CLIMBING;
        } else {
          chassis.setSpeedsRobotRel((new ChassisSpeeds(ClimbConstants.DRIVE_SPEED, 0, 0)));
        }
        break;

      case CLIMBING:
        climbSubsystem.setPosition(ClimbConstants.BALANCE_START_DISTANCE, ClimbConstants.BALANCE_START_DISTANCE);
        if (climbSubsystem.isAtTarget(ClimbConstants.BALANCE_START_DISTANCE)) {
          stage = ClimbStage.BALANCING;
        }
        break;

      case BALANCING:
        climbSubsystem.balance(ClimbConstants.BALANCE_START_DISTANCE);
        if (climbSubsystem.isBalanced()) {
          stage = ClimbStage.FINISHED;
        }
        break;

      case FINISHED:
        climbSubsystem.stop();
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    climbSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == ClimbStage.FINISHED;
  }
}
