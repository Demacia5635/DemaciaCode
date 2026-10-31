// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.commands;

import frc.demacia.path.points.Point;
import edu.wpi.first.math.MathUtil;
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

import static frc.demacia.path.points.Point.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class followPathCommand extends Command {
  private static double ALOWED_ARROR = 0.1;
  private static double MAX_VELOCITY = 1.5;
  private static double MAX_ANGLE_VELOCITY = 4;
  private static double K_P = 7;
  private static double ANGLE_K_P = 4;

  private Chassis chassis;
  private Path path;
  private Segmant currentSegmant;
  private Point nextAnglePoint;
  private int nextAnglePointIndex;
  private double nextVelocityPoint;
  private double fixedDistance;

  private int index;
  private int pointIndex;

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
    pointIndex = 0;
    updateSegmant();
    updateNextAnglePoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = RobotPose.getInstance().getEstimatedPose();
    Point robotPoint = new Point(robotPose.getX(), robotPose.getY());
    double velocity = MAX_VELOCITY;
    double angleVelocity = MAX_ANGLE_VELOCITY;

    if (Point.distance(robotPoint, currentSegmant.getEndPoint()) < ALOWED_ARROR) {
      index++;
      if (index >= path.getSegmants().length) {
        return;
      }

      updateSegmant();
    };

    if (Point.distance(robotPoint, path.getInitinalPoints()[pointIndex + 1]) < ALOWED_ARROR) {
      pointIndex++;
      updateNextAnglePoint();
    }

    Log.log(currentSegmant.getDistanceFromSegmant(robotPoint));

    double xVel = 0;
    double yVel = 0;
    if (currentSegmant instanceof LineSegmant) {
      double alpha = Point.atan2(currentSegmant.getEndPoint(), currentSegmant.getStartPoint());
      xVel = velocity*Math.cos(alpha);
      yVel = velocity*Math.sin(alpha);

      double distanceFromLine = currentSegmant.getDistanceFromSegmant(robotPoint);
      double sigh = sigh(currentSegmant.getStartPoint(), robotPoint, currentSegmant.getEndPoint());
      xVel += sigh*K_P*distanceFromLine*Math.sin(alpha);
      yVel -= sigh*K_P*distanceFromLine*Math.cos(alpha);
    } else if (currentSegmant instanceof CircleSegmant) {
      double alpha = Math.PI/2 + Point.atan2(((CircleSegmant) currentSegmant).getCirclePoint().getMiddlePoint(), robotPoint);
      double sign = sigh(currentSegmant.getStartPoint(), ((CircleSegmant) currentSegmant).getCirclePoint(), currentSegmant.getEndPoint());
      xVel = sign*velocity*Math.cos(alpha);
      yVel = sign*velocity*Math.sin(alpha);

      double distanceFromCircle = currentSegmant.getDistanceFromSegmant(robotPoint);
      double fixSign = Math.signum(((CircleSegmant) currentSegmant).getCirclePoint().getRadius()-distanceFromCircle);
      xVel += fixSign*K_P*distanceFromCircle*Math.sin(alpha);
      yVel -= fixSign*K_P*distanceFromCircle*Math.cos(alpha);
    }

    double distanceFromNextAnglePoint = (fixedDistance + currentSegmant.getRemainingDistance(robotPoint));

    double omega = MathUtil.clamp(ANGLE_K_P * MathUtil.angleModulus(nextAnglePoint.getAngle().getRadians() - chassis.getGyroAngle().getRadians()), -MAX_ANGLE_VELOCITY, MAX_ANGLE_VELOCITY);

    chassis.setSpeedsFieldRel(new ChassisSpeeds(xVel, yVel, omega));
  }

  private void updateSegmant() {
    currentSegmant = path.getSegmants()[index];

    fixedDistance = 0;
    for (int i = index+1; i < 2*nextAnglePointIndex-3; i++) {
      fixedDistance += path.getSegmants()[i].getDistance();
    }
    if (nextAnglePointIndex != path.getInitinalPoints().length - 1 && 2*nextAnglePointIndex - 1 >= 0) {
      fixedDistance -= path.getSegmants()[2*nextAnglePointIndex - 1].getDistance() / 2;
    }
  }

  private void updateNextAnglePoint() {
    boolean found = false;
    for (int i = pointIndex + 1; i < path.getInitinalPoints().length; i++) {
      if (path.getInitinalPoints()[i].getAngle() != null) {
        nextAnglePointIndex = i;
        nextAnglePoint = path.getInitinalPoints()[i];
        found = true;
        break;
      }
    }
    if (!found) {
      nextAnglePointIndex = path.getInitinalPoints().length - 1;
      nextAnglePoint = path.getInitinalPoints()[path.getInitinalPoints().length - 1];
      nextAnglePoint.setAngle((path.getInitinalPoints()[pointIndex].getAngle() != null) ? 
      path.getInitinalPoints()[pointIndex].getAngle() : chassis.getGyroAngle());
    }
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
