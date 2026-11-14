package frc.demacia.path.calculator;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.utils.log.Log;

public class LegCalculator {

    // returns the arc entry and exit points, both with the given robot heading
    public static List<Pose2d> returnPoint(Translation2d p1, Translation2d p2, ArcSegment arc, double radius, Rotation2d heading){
        Translation2d distance = arc.getCenter().minus(p1);

        //the base angle in 90 degrees triangle
        double alpha = Math.asin(radius/(distance.getNorm()));

        Translation2d vector1 = distance.rotateBy(new Rotation2d(arc.getIsLeft() ? -alpha : alpha));

        Translation2d vector2 = vector1.div(vector1.getNorm()).times(Math.cos(alpha) * distance.getNorm());

        Translation2d pp1 = p1.plus(vector2);

        Translation2d vector3 = pp1.minus(arc.getCenter());
        Translation2d vector4 = p2.minus(arc.getCenter());

        Translation2d pp2 = vector4.rotateBy(vector4.getAngle().minus(vector3.getAngle())).plus(arc.getCenter());
        Log.log("p1: " + p1 + " vector2: " + vector2 + " vector1: " + vector1 + " arc: " + arc);
        List<Pose2d> points = new ArrayList<>();
        points.add(new Pose2d(pp1, heading));
        points.add(new Pose2d(pp2, heading));
        return points;
    }
}
