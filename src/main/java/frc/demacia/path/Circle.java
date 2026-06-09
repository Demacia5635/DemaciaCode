package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.log.LogManager;

public class Circle {
    public Translation2d center;
    public double radius;
    public boolean isLeft;
    public double angleDiff;

    // Track accumulated distance inside this circle
    private double totalDistanceTraveled = 0.0;
    private Pose2d lastRobotPose = null;

    private DemaciaTrapezoid trapezoid = new DemaciaTrapezoid(3, 0);

    public Circle(Translation2d center, double radius, boolean isLeft, double angleDiff) {
        this.center = center;
        this.radius = radius;
        this.isLeft = isLeft;
        this.angleDiff = angleDiff;
    }

    public double getDist(){
        // LogManager.log("")
        return radius * Math.abs(angleDiff);
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

    public double getVel(){
        double totalArcDist = getDist();
        double distLeft = totalArcDist - totalDistanceTraveled;
        LogManager.log("dist left" + distLeft + " total dist traaveld"+ totalDistanceTraveled + "total arch dist" + totalArcDist);
        if (distLeft < 0) distLeft = 0;
        return trapezoid.calculate(totalDistanceTraveled, totalArcDist, 0);
    }

    /**
     * Calculates the target heading for the robot inside the circle path.
     * Formula: heading = 2 * currentAngle + baseVector.angle()
     */
    public Rotation2d getTargetHeading(Rotation2d currentAngle, Translation2d baseVector) {
        double headingRadians = 2 * currentAngle.getRadians() + baseVector.getAngle().getRadians();
        return new Rotation2d(headingRadians);
    }

    public void resetTracker() {
        this.totalDistanceTraveled = 0.0;
        this.lastRobotPose = null;
    }

    public boolean isPointInside(Pose2d robotPose) {
        double dx = robotPose.getX() - this.center.getX();
        double dy = robotPose.getY() - this.center.getY();
        return Math.sqrt(dx * dx + dy * dy) <= radius;
    }
}