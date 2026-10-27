package frc.demacia.path.points;

import edu.wpi.first.math.geometry.Pose2d;

public class Point {
    protected double pointX;
    protected double pointY;
    protected double angle;

    public Point(double pointX, double pointY, double angle) {
        this.pointX = pointX;
        this.pointY = pointY;
        this.angle = angle;
    }

    public Point(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getRotation().getRadians());
    }

    public void setPointX(double pointX) {
        this.pointX = pointX;
    }

    public void setPointY(double pointY) {
        this.pointY = pointY;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double getPointX() {
        return pointX;
    }

    public double getPointY() {
        return pointY;
    }

    public double getAngle() {
        return angle;
    }

    public static double distance(Point point1, Point point2) {
        return Math.sqrt(Math.pow(point1.getPointX() - point2.getPointX(), 2) + Math.pow(point1.getPointY() - point2.getPointY(), 2));
    }

    public static double distance(Pose2d point1, Point point2) {
        return Math.sqrt(Math.pow(point1.getX() - point2.getPointX(), 2) + Math.pow(point1.getY() - point2.getPointY(), 2));
    }

    public static double sigh(Point point1, Point point2, Point point3) {
        return Math.signum(
            (point1.getPointX() - point2.getPointX())*(point3.getPointY() - point2.getPointY()) - 
            (point1.getPointY() - point2.getPointY())*(point3.getPointX() - point2.getPointX())
        );
    }

    public static double atan2(Point point1, Point point2) {
        return Math.atan2(point1.getPointY() - point2.getPointY(), point1.getPointX() - point2.getPointX());
    }
}
