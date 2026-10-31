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
        return distance(robotPose, getCirclePoint().getCenterPoint()) - getCirclePoint().getRadius();
    }

    @Override
    public double getRemainingDistance(Point robotPose) {
        Point center = circlePoint.getCenterPoint();
        double angle = sigh(startPoint, circlePoint, endPoint) * (atan2(robotPose, center) - atan2(endPoint, center));
        angle -= 2*Math.PI*Math.floor(angle/(2*Math.PI));
        return circlePoint.getRadius()*angle;
    }
}
