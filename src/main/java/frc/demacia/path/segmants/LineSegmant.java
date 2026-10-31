package frc.demacia.path.segmants;

import frc.demacia.path.points.Point;
import static frc.demacia.path.points.Point.*;

public class LineSegmant extends Segmant {

    public LineSegmant(Point startPoint, Point endPoint) {
        super(startPoint, endPoint);
        distance = getRemainingDistance(startPoint);
    }

    @Override
    public double getDistanceFromSegmant(Point robotPose) {
        double a = distance(robotPose, endPoint);
        double b = distance(robotPose, startPoint);
        double c = distance(startPoint, endPoint);
        
        return Math.sqrt(Math.max(0, Math.pow(a, 2) - 
            Math.pow((Math.pow(a, 2) + Math.pow(c, 2) - Math.pow(b, 2)) / (2*c), 2)));
    }

    @Override
    public double getRemainingDistance(Point robotPose) {
        return distance(robotPose, endPoint);
    }
}
