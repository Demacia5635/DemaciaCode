package frc.demacia.path.points;

public class CirclePoint extends Point{
    private Point middlePoint;
    private double radius;

    public CirclePoint(double pointX, double pointY) {
        super(pointX, pointY);
        middlePoint = new Point(0, 0);
    }

    public CirclePoint(Point point) {
        this(point.getPointX(), point.getPointY());
        if (point.angle != null) {
            withAngle(point.angle);
        }
        if (point.velocity != null) {
            withVelocity(point.velocity);
        }
        middlePoint = new Point(0, 0);
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
