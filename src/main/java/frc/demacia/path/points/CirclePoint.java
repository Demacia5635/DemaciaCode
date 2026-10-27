package frc.demacia.path.points;

import edu.wpi.first.math.geometry.Pose2d;

public class CirclePoint extends Point{
    private Point middlePoint;
    private double radius;

    public CirclePoint(double pointX, double pointY, double angle) {
        super(pointX, pointY, angle);
        middlePoint = new Point(0, 0, 0);
    }

    public CirclePoint(Pose2d pose2d) {
        super(pose2d);
        middlePoint = new Point(0, 0, 0);
    }

    public void setCircleX(double circleX) {
        middlePoint.setPointX(circleX);
    }

    public void setCircleY(double circleY) {
        middlePoint.setPointY(circleY);
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public Point getMiddlePoint() {
        return middlePoint;
    }

    public double getCircleX() {
        return middlePoint.getPointX();
    }

    public double getCircleY() {
        return middlePoint.getPointY();
    }

    public double getRadius() {
        return radius;
    }
}
