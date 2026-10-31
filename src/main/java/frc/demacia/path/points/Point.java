package frc.demacia.path.points;

import edu.wpi.first.math.geometry.Rotation2d;

public class Point {
    protected double pointX;
    protected double pointY;
    protected Rotation2d angle;
    protected Double velocity;

    public Point(double pointX, double pointY) {
        this.pointX = pointX;
        this.pointY = pointY;
        angle = null;
        velocity = null;
    }

    public Point withAngle(Rotation2d angle) {
        setAngle(angle);
        return this;
    }

    public Point withVelocity(Double velocity) {
        setVelocity(velocity);
        return this;
    }

    public void setPointX(double pointX) {
        this.pointX = pointX;
    }

    public void setPointY(double pointY) {
        this.pointY = pointY;
    }

    public void setAngle(Rotation2d angle) {
        this.angle = angle;
    }

    public void setVelocity(Double velocity) {
        this.velocity = velocity;
    }

    public double getPointX() {
        return pointX;
    }

    public double getPointY() {
        return pointY;
    }

    public Rotation2d getAngle() {
        return angle;
    }

    public Double getVelocity() {
        return velocity;
    }

    public static double distance(Point point1, Point point2) {
        return Math.sqrt(Math.pow(point1.getPointX() - point2.getPointX(), 2) + Math.pow(point1.getPointY() - point2.getPointY(), 2));
    }

    public static double sigh(Point point1, Point point2, Point point3) {
        return Math.signum(0.5 + Math.signum(
            (point1.getPointX() - point2.getPointX())*(point3.getPointY() - point2.getPointY()) - 
            (point1.getPointY() - point2.getPointY())*(point3.getPointX() - point2.getPointX())
        ));
    }

    public static double atan2(Point point1, Point point2) {
        return Math.atan2(point1.getPointY() - point2.getPointY(), point1.getPointX() - point2.getPointX());
    }
}
