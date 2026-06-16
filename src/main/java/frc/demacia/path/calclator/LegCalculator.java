package frc.demacia.path.calclator;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.utils.log.LogManager;

/**
 * Static helpers for computing tangent points between points and circles.
 * Four cases are handled:
 * 1. Circle -> Circle (Overloaded wrapper accepting Circle objects)
 * 2. Point -> Circle  (First and last leg of the path)
 * 3. Circle -> Circle, same turn direction  (Parallel tangent)
 * 4. Circle -> Circle, opposite turn direction  (Cross tangent)
 */
public class LegCalculator {

    /**
     * Automatically detects whether the two circles turn in the same or opposite
     * directions, and returns the tangent poses with the correct driving heading.
     *
     * @param c1 the starting circle
     * @param c2 the ending circle
     * @return an array containing [Pose2d tangent on circle 1, Pose2d tangent on circle 2]
     */
    // public static Pose2d[] circleToCircleTangents(Circle c1, Circle c2) {
    //     if (c1.isLeft == c2.isLeft) {
    //         return sameTurnTangents(c1.center, c2.center, c1.radius, c1.isLeft);
    //     } else {
    //         return oppositeTurnTangents(c1.center, c2.center, c1.radius, c1.isLeft);
    //     }
    // }

    public static List<Translation2d> returnPoint(Translation2d p1, Translation2d p2, ArcSegment arc, double radius){
        Translation2d distance = arc.getCenter().minus(p1);
        double alpha = /*Math.acos*/Math.asin(radius/(distance.getNorm()));
        // double beta = 90 - alpha; //180 - (alpha + 90);
        Translation2d vector1 = distance.rotateBy(new Rotation2d(alpha));
        Translation2d vector2 = vector1.div(vector1.getNorm()).times(Math.cos(alpha) * distance.getNorm());

        Translation2d pp1 = p1.plus(vector2);

        Translation2d vector3 = pp1.minus(arc.getCenter());
        Translation2d vector4 = p2.minus(arc.getCenter());

        Translation2d pp2 = vector4.rotateBy(vector4.getAngle().minus(vector3.getAngle())).plus(arc.getCenter());

        LogManager.log("p1: " + p1 + " vector2: " + vector2 + " vector1: " + vector1 + " arc: " + arc);
        return new ArrayList<>(){
            {
                add(pp1);
                add(pp2);
            }
        };
        // Pose2d[]{
        //     new Pose2d(p1, vector3.getAngle()),
        //     new Pose2d(vector2, vector3.getAngle())
        // }.asLi;
    }


    /**
     * Tangent point from an external point P1 to a circle.
     * The tangent-to-center angle is always 90°, so:
     * alpha = acos(r / d)
     * The sign of alpha depends on the turn direction.
     *
     * @param p1         the external point
     * @param center     circle center
     * @param radius     circle radius
     * @param isLeftTurn turn direction of the circle
     * @return the tangent point on the circle
     */
    public static Translation2d pointToCircleTangent(Pose2d p1,
                                                     Translation2d center,
                                                     double radius,
                                                     boolean isLeftTurn) {
        Translation2d vec = p1.getTranslation().minus(center);
        double d = vec.getNorm();
        double baseAngle = vec.getAngle().getRadians();
        double alpha = Math.acos(radius / d);
        // Subtract alpha for left turn, add for right turn
        double angle = baseAngle + (isLeftTurn ? -alpha : alpha);

        return center.plus(new Translation2d(radius * Math.cos(angle),
                                             radius * Math.sin(angle)));
    }

    /**
     * Tangent points between two circles with the SAME turn direction.
     * The tangent line is perpendicular to the vector between centers,
     * offset by radius on the same side for both circles.
     *
     * @param center1    center of circle 1
     * @param center2    center of circle 2
     * @param radius     shared radius
     * @param isLeftTurn shared turn direction
     * @return an array containing [Pose2d tangent on circle 1, Pose2d tangent on circle 2]
     */
    public static Pose2d[] sameTurnTangents(Translation2d center1,
                                            Translation2d center2,
                                            double radius,
                                            boolean isLeftTurn) {
        Translation2d vec = center2.minus(center1);
        double baseAngle = vec.getAngle().getRadians();
        // Perpendicular to the center-to-center vector
        double angle = baseAngle + (isLeftTurn ? -Math.PI / 2 : Math.PI / 2);

        Translation2d offset = new Translation2d(radius * Math.cos(angle),
                                                 radius * Math.sin(angle));
        
        Translation2d t1 = center1.plus(offset);
        Translation2d t2 = center2.plus(offset);
        
        // Calculate the actual driving direction along the tangent line
        Rotation2d heading = t2.minus(t1).getAngle();

        return new Pose2d[]{
                new Pose2d(t1, heading),
                new Pose2d(t2, heading)
        };
    }

    /**
     * Tangent points between two circles with OPPOSITE turn directions.
     * Uses the cross tangent formula:
     * a = acos(2r / d)
     * The offset vector is added to center1 and subtracted from center2.
     *
     * @param center1    center of circle 1
     * @param center2    center of circle 2
     * @param radius     shared radius
     * @param isLeftTurn turn direction of circle 1
     * @return an array containing [Pose2d tangent on circle 1, Pose2d tangent on circle 2]
     */
    public static Pose2d[] oppositeTurnTangents(Translation2d center1,
                                                Translation2d center2,
                                                double radius,
                                                boolean isLeftTurn) {
        Translation2d vec = center2.minus(center1);
        double d = vec.getNorm();
        double baseAngle = vec.getAngle().getRadians();
        double a = Math.acos(radius * 2 / d);
        // Add a for left-to-right transition, subtract for right-to-left
        double angle = baseAngle + (isLeftTurn ? +a : -a);

        Translation2d offset = new Translation2d(radius * Math.cos(angle),
                                                 radius * Math.sin(angle));
        
        Translation2d t1 = center1.plus(offset);
        Translation2d t2 = center2.minus(offset);
        
        // Calculate the actual driving direction along the tangent line
        Rotation2d heading = t2.minus(t1).getAngle();

        return new Pose2d[]{
                new Pose2d(t1, heading),
                new Pose2d(t2, heading)
        };
    }
}