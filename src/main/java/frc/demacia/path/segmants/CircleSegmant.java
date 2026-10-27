package frc.demacia.path.segmants;

import frc.demacia.path.points.CirclePoint;
import frc.demacia.path.points.Point;
import static frc.demacia.path.points.Point.*;

import edu.wpi.first.math.geometry.Pose2d;

public class CircleSegmant extends Segmant {
    protected CirclePoint circlePoint;

    public CircleSegmant(Point startPoing, Point endPoing, CirclePoint circlePoint) {
        super(startPoing, endPoing);
        this.circlePoint = circlePoint;
        distance = circlePoint.getRadius() *
            Math.acos((Math.pow(distance(startPoing, circlePoint.getMiddlePoint()), 2) + 
                Math.pow(distance(endPoing, circlePoint.getMiddlePoint()), 2) - 
                Math.pow(distance(startPoing, endPoing), 2)) / 
                (2*distance(startPoing, circlePoint.getMiddlePoint())*distance(endPoing, circlePoint.getMiddlePoint())));
    }

    public CirclePoint getCirclePoint() {
        return circlePoint;
    }

    public double distanceFromCircle(Pose2d pose) {
        double a = pose.getY() - getCirclePoint().getMiddlePoint().getPointY();
        double b = pose.getX() - getCirclePoint().getMiddlePoint().getPointX();

        return Math.hypot(a, b) - getCirclePoint().getRadius();
    }
}
