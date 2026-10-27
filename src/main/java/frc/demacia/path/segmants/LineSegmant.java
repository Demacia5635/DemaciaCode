package frc.demacia.path.segmants;

import frc.demacia.path.points.Point;

public class LineSegmant extends Segmant {

    public LineSegmant(Point startPoing, Point endPoing) {
        super(startPoing, endPoing);
        distance = Point.distance(startPoing, endPoing);
    }
}
