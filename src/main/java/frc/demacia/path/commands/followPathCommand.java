// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.commands;

import frc.demacia.path.points.Point;

import static frc.demacia.path.points.Point.sigh;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.Path;
import frc.demacia.path.segmants.CircleSegmant;
import frc.demacia.path.segmants.LineSegmant;
import frc.demacia.path.segmants.Segmant;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.Log;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class followPathCommand extends Command {
  private static double ALOWED_ARROR = 0.1;
  private static double MAX_VELOCITY = 0.5;
  private static double K_P = 2;

  private Chassis chassis;
  private Path path;
  private Segmant currentSegmant;

  private int index;

  /** Creates a new followPathCommand. */
  public followPathCommand(Path path) {
    chassis = Chassis.getInstance();
    this.path = path;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index = 0;
    currentSegmant = path.getSegmants()[index];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = RobotPose.getInstance().getEstimatedPose();

    if (Point.distance(robotPose, currentSegmant.getEndPoint()) < ALOWED_ARROR) {
      index++;
      Log.log(index + " index");
      Log.log(robotPose.getX() + " robotPoseXXXXXX");
      Log.log(robotPose.getY() + " robotPoseYYYYYY");
      Log.log(currentSegmant.getEndPoint().getPointX() + " currentSegmantXXXXXXXXXX");
      Log.log(currentSegmant.getEndPoint().getPointY() + " currentSegmantYYYYYYYYYY");
      if (index >= path.getSegmants().length) {
        return;
      }

      currentSegmant = path.getSegmants()[index];
    }

    double xVel = 0;
    double yVel = 0;
    if (currentSegmant instanceof LineSegmant) {
      double alpha = Point.atan2(currentSegmant.getEndPoint(), currentSegmant.getStartPoint());
      xVel = MAX_VELOCITY*Math.cos(alpha);
      yVel = MAX_VELOCITY*Math.sin(alpha);

      double distanceFromLine = ((LineSegmant) currentSegmant).distanceFromLine(robotPose);
      double sigh = sigh(currentSegmant.getStartPoint(), new Point(robotPose), currentSegmant.getEndPoint());
      xVel += sigh*K_P*distanceFromLine*Math.sin(alpha);
      yVel -= sigh*K_P*distanceFromLine*Math.cos(alpha);
    } else if (currentSegmant instanceof CircleSegmant) {
      double cx = ((CircleSegmant) currentSegmant).getCirclePoint().getMiddlePoint().getPointX();
      double cy = ((CircleSegmant) currentSegmant).getCirclePoint().getMiddlePoint().getPointY();
      Segmant prev = path.getSegmants()[index - 1];
      double rx = currentSegmant.getStartPoint().getPointX() - cx;
      double ry = currentSegmant.getStartPoint().getPointY() - cy;
      double tx = prev.getEndPoint().getPointX() - prev.getStartPoint().getPointX();
      double ty = prev.getEndPoint().getPointY() - prev.getStartPoint().getPointY();
      double dir = -Math.signum(rx*ty - ry*tx);
      double alpha = Point.atan2(((CircleSegmant) currentSegmant).getCirclePoint().getMiddlePoint(), new Point(robotPose)) + dir*Math.PI/2;
      xVel = MAX_VELOCITY*Math.cos(alpha);
      yVel = MAX_VELOCITY*Math.sin(alpha);

      double distanceFromCircle = ((CircleSegmant) currentSegmant).distanceFromCircle(robotPose);
      double dx = ((CircleSegmant) currentSegmant).getCirclePoint().getMiddlePoint().getPointX() - robotPose.getX();
      double dy = ((CircleSegmant) currentSegmant).getCirclePoint().getMiddlePoint().getPointY() - robotPose.getY();
      double dist = Math.hypot(dx, dy);
      xVel += K_P*distanceFromCircle*dx/dist;
      yVel += K_P*distanceFromCircle*dy/dist;
    }

    chassis.setSpeedsFieldRel(new ChassisSpeeds(xVel, yVel, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index >= path.getSegmants().length;
  }
}
