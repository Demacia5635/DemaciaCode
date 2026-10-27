package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.path.points.CirclePoint;
import frc.demacia.path.points.Point;
import frc.demacia.path.segmants.CircleSegmant;
import frc.demacia.path.segmants.LineSegmant;
import frc.demacia.path.segmants.Segmant;

import static frc.demacia.path.points.Point.*;

public class Path {
    static public double RADIUS = 4;

    private Pose2d[] initinalPoints;
    private Point[] tempereryPoints;
    private Segmant[] segmants;
    private int length;

    public Path(Pose2d... initinalPoints) {
        this.initinalPoints = initinalPoints;
        length = initinalPoints.length;

        createTempereryPoints();

        createSegmants();
    }

    public void createTempereryPoints() {
        tempereryPoints = new Point[length];
        for (int i = 0; i < length; i++) {
            if (i == 0 || i == length - 1) {
                tempereryPoints[i] = new Point(initinalPoints[i]);
            } else {
                tempereryPoints[i] = new CirclePoint(initinalPoints[i]);
            }
        }
        for (int i = 1; i < length - 1; i++) {
            double radius;
            radius = Math.min(Math.min(RADIUS, distance(tempereryPoints[i - 1], tempereryPoints[i])/((i == 1) ? 2 : 4)), distance(tempereryPoints[i], tempereryPoints[i + 1])/((i == length - 2) ? 2 : 4));
            ((CirclePoint) tempereryPoints[i]).setRadius(radius);

            setCircle((CirclePoint) tempereryPoints[i], tempereryPoints[i - 1], tempereryPoints[i + 1]);
        }
    }

    public void setCircle(CirclePoint circlePoint, Point previousPoint, Point nextPoint) {
        double alpha = Math.acos((Math.pow(distance(previousPoint, circlePoint), 2) + 
            Math.pow(distance(circlePoint, nextPoint), 2) - 
            Math.pow(distance(previousPoint, nextPoint), 2)) / 
            (2*distance(previousPoint, circlePoint)*distance(circlePoint, nextPoint))) / 2;
        double sign = sigh(previousPoint, circlePoint, nextPoint);
        double teta = atan2(previousPoint, circlePoint) + sign * alpha;

        double circleX = circlePoint.getPointX() + circlePoint.getRadius()*Math.cos(teta);
        double circleY = circlePoint.getPointY() + circlePoint.getRadius()*Math.sin(teta);

        circlePoint.setCircleX(circleX);
        circlePoint.setCircleY(circleY);
    }

    public void createSegmants() {
        if (length < 2) {
            return;
        }
        if (length == 2) {
            segmants = new Segmant[] {new LineSegmant(tempereryPoints[0], tempereryPoints[1])};
            return;
        }

        segmants = new Segmant[2*length - 3];

        for (int i = 0; i < length - 1; i++) {
            if (i == 0) {
                segmants[i] = Segmant.createLineSegmant(tempereryPoints[i], (CirclePoint) tempereryPoints[i+1], tempereryPoints[i+2]);
            } else if (i == length - 2) {
                segmants[2*i] = Segmant.createLineSegmant(tempereryPoints[i+1], (CirclePoint) tempereryPoints[i], tempereryPoints[i-1]);
                segmants[2*i] = new LineSegmant(segmants[2*i].getEndPoint(), segmants[2*i].getStartPoint());
                segmants[2*i-1] = new CircleSegmant(segmants[2*i - 2].getEndPoint(), segmants[2*i].getStartPoint(), (CirclePoint) tempereryPoints[i]);
            } else {
                segmants[2*i] = Segmant.createLineSegmant(tempereryPoints[i-1], (CirclePoint) tempereryPoints[i], (CirclePoint) tempereryPoints[i+1], tempereryPoints[i+2]);
                segmants[2*i - 1] = new CircleSegmant(segmants[2*i - 2].getEndPoint(), segmants[2*i].getStartPoint(), (CirclePoint) tempereryPoints[i]);
            }
        }
    }

    public Segmant[] getSegmants() {
        return segmants;
    }
}
