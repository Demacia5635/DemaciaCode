package frc.demacia.path.segmants;

import frc.demacia.path.points.CirclePoint;
import frc.demacia.path.points.Point;
import static frc.demacia.path.points.Point.*;

public class CircleSegmant extends Segmant {
    protected CirclePoint circlePoint;

    public CircleSegmant(Point startPoint, Point endPoint, CirclePoint circlePoint) {
        super(startPoint, endPoint);
        this.circlePoint = circlePoint;
        distance = getRemainingDistance(startPoint);
    }

    public CirclePoint getCirclePoint() {
        return circlePoint;
    }

    @Override
    public double getDistanceFromSegmant(Point robotPose) {
        return distance(robotPose, getCirclePoint().getMiddlePoint()) - getCirclePoint().getRadius();
    }

    @Override
    public double getRemainingDistance(Point robotPose) {
        return circlePoint.getRadius() *
        Math.acos((Math.pow(distance(robotPose, circlePoint.getMiddlePoint()), 2) + 
            Math.pow(distance(endPoint, circlePoint.getMiddlePoint()), 2) - 
            Math.pow(distance(robotPose, endPoint), 2)) / 
            (2*distance(robotPose, circlePoint.getMiddlePoint())*distance(endPoint, circlePoint.getMiddlePoint())));
    }
}
