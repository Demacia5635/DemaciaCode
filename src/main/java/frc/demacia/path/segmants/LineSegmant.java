package frc.demacia.path.segmants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.path.points.Point;
import static frc.demacia.path.points.Point.*;

public class LineSegmant extends Segmant {

    public LineSegmant(Point startPoint, Point endPoint) {
        super(startPoint, endPoint);
        distance = Point.distance(startPoint, endPoint);
    }

    public double distanceFromLine(Pose2d pose) {
        double a = distance(pose, endPoint);
        double b = distance(pose, startPoint);
        double c = distance(startPoint, endPoint);
        
        return Math.sqrt(Math.max(0, Math.pow(a, 2) - 
            Math.pow((Math.pow(a, 2) + Math.pow(c, 2) - Math.pow(b, 2)) / (2*c), 2)));
    }
}
