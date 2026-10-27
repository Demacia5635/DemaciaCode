package frc.demacia.path.segmants;

import frc.demacia.path.points.CirclePoint;
import frc.demacia.path.points.Point;
import static frc.demacia.path.points.Point.*;

public class CircleSegmant extends Segmant {

    public CircleSegmant(Point startPoing, Point endPoing, CirclePoint circlePoint) {
        super(startPoing, endPoing);
        distance = circlePoint.getRadius() *
            Math.acos((Math.pow(distance(startPoing, circlePoint.getMiddlePoint()), 2) + 
                Math.pow(distance(endPoing, circlePoint.getMiddlePoint()), 2) - 
                Math.pow(distance(startPoing, endPoing), 2)) / 
                (2*distance(startPoing, circlePoint.getMiddlePoint())*distance(endPoing, circlePoint.getMiddlePoint())));
    }
}
