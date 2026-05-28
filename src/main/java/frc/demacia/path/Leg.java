package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Leg {
    private Translation2d start;
    private Translation2d end;
    private Circle startCircle;
    private Circle endCircle;

    // Track accumulated distance inside this straight segment
    private double totalDistanceTraveled = 0.0;
    private Pose2d lastRobotPose = null;

    private DemaciaTrapezoid trapezoid = new DemaciaTrapezoid(3, 0);

    public Leg(Circle startCircle, Circle endCircle) {
        this.startCircle = startCircle;
        this.endCircle = endCircle;
        Translation2d startToEnd = endCircle.center.minus(startCircle.center);
        Rotation2d startAngleChange;
        Rotation2d endAngleChange;

        if (startCircle.isLeft == endCircle.isLeft) {
            startAngleChange = new Rotation2d(Math.PI / 2);
            endAngleChange = new Rotation2d(Math.PI / 2);
        } else {
            startAngleChange = new Rotation2d(Math.acos(2 * (startCircle.radius / startToEnd.getNorm())));
            endAngleChange = new Rotation2d(Math.acos(2 * (endCircle.radius / startToEnd.getNorm())));
        }

        startAngleChange = startAngleChange.times((startCircle.isLeft ? -1 : 1));
        endAngleChange = endAngleChange.times((endCircle.isLeft ? -1 : 1));

        this.start = startCircle.center.plus(new Translation2d(startCircle.radius, startToEnd.getAngle().rotateBy(startAngleChange)));
        this.end = endCircle.center.plus(new Translation2d(endCircle.radius, startToEnd.getAngle().rotateBy(endAngleChange)));
    }

    public double getDist() {
        return start.getDistance(end);
    }

    public void updateDistance(Pose2d currentPose) {
        if (lastRobotPose == null) {
            lastRobotPose = currentPose;
            return;
        }
        double dx = currentPose.getX() - lastRobotPose.getX();
        double dy = currentPose.getY() - lastRobotPose.getY();
        totalDistanceTraveled += Math.hypot(dx, dy);
        lastRobotPose = currentPose;
    }

    public double getVel() {
        double totalLegDist = getDist();
        double distLeft = totalLegDist - totalDistanceTraveled;
        if (distLeft < 0) distLeft = 0;
        return trapezoid.calculate(totalDistanceTraveled, totalLegDist, 0);
    }

    /**
     * Calculates the target heading for the robot while driving on a straight line.
     */
    public Rotation2d getTargetHeading(Rotation2d currentAngle, Translation2d baseVector) {
        double headingRadians = 2 * currentAngle.getRadians() + baseVector.getAngle().getRadians();
        return new Rotation2d(headingRadians);
    }

    public void resetTracker() {
        this.totalDistanceTraveled = 0.0;
        this.lastRobotPose = null;
    }

    public Translation2d getStart() { return start; }
    public Translation2d getEnd() { return end; }
    public Circle getStartCircle() { return startCircle; }
    public Circle getEndCircle() { return endCircle; }

    public boolean isRobotInside(Pose2d robotPose) {
        return startCircle.isPointInside(robotPose) || endCircle.isPointInside(robotPose);
    }
}