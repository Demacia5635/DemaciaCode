package frc.demacia.path.segmants;

import frc.demacia.path.points.CirclePoint;
import frc.demacia.path.points.Point;
import static frc.demacia.path.points.Point.*;

public class Segmant {
    protected Point startPoint;
    protected Point endPoint;
    protected double distance;

    public Segmant(Point startPoing, Point endPoing) {
        this.startPoint = startPoing;
        this.endPoint = endPoing;
    }

    public double getDistance() {
        return distance;
    }

    public Point getStartPoint() {
        return startPoint;
    }

    public Point getEndPoint() {
        return endPoint;
    }

    public static LineSegmant createLineSegmant(Point point1, CirclePoint point2, Point point3) {
        double alpha = Math.asin(point2.getRadius() / distance(point1, point2.getMiddlePoint()));
        double sign = sigh(point1, point2, point3);
        double teta = atan2(point2.getMiddlePoint(), point1) + sign * alpha;
        double dist = Math.sqrt(Math.pow(distance(point1, point2.getMiddlePoint()), 2) - 
            Math.pow(point2.getRadius(), 2));

        double segmantPointX = point1.getPointX() + dist*Math.cos(teta);
        double segmantPointY = point1.getPointY() + dist*Math.sin(teta);

        return new LineSegmant(point1, new Point(segmantPointX, segmantPointY, 0));
    }

    public static LineSegmant createLineSegmant(Point point1, CirclePoint point2, CirclePoint point3, Point point4) {
        double sign2 = -sigh(point1, point2, point3)*sigh(point2, point3, point4);
        double alpha = sign2 * Math.acos((point3.getRadius() + sign2*point2.getRadius()) / distance(point2.getMiddlePoint(), point3.getMiddlePoint()));
        double sign = sigh(point1, point2, point3);
        double teta = atan2(point3.getMiddlePoint(), point2.getMiddlePoint()) + sign * alpha;

        double segmantPoint1X = point2.getMiddlePoint().getPointX() + sign2*point2.getRadius()*Math.cos(teta);
        double segmantPoint1Y = point2.getMiddlePoint().getPointY() + sign2*point2.getRadius()*Math.sin(teta);
        double segmantPoint2X = point3.getMiddlePoint().getPointX() - point3.getRadius()*Math.cos(teta);
        double segmantPoint2Y = point3.getMiddlePoint().getPointY() - point3.getRadius()*Math.sin(teta);

        return new LineSegmant(new Point(segmantPoint1X, segmantPoint1Y, 0), 
            new Point(segmantPoint2X, segmantPoint2Y, 0));
    }
}
