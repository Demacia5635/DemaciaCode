package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Static helpers for computing tangent points between points and circles.
 * Three cases are handled:
 *   1. Point -> Circle  (first and last leg of the path)
 *   2. Circle -> Circle, same turn direction  (parallel tangent)
 *   3. Circle -> Circle, opposite turn direction  (cross tangent)
 */
public class LegCalculator {

    /**
     * Tangent point from an external point P1 to a circle.
     * The tangent-to-center angle is always 90°, so:
     *   alpha = acos(r / d)
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
     * @return [tangent1 on circle1, tangent2 on circle2]
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
        return new Pose2d[]{
                new Pose2d(center1.plus(offset).getX(), center1.plus(offset).getY(), center1.plus(offset).getAngle()),
                new Pose2d(center2.plus(offset).getX(), center2.plus(offset).getY(), center2.plus(offset).getAngle())   // same offset — parallel tangent
        };
    }

    /**
     * Tangent points between two circles with OPPOSITE turn directions.
     * Uses the cross tangent formula:
     *   a = acos(2r / d)
     * The offset vector is added to center1 and subtracted from center2.
     *
     * @param center1    center of circle 1
     * @param center2    center of circle 2
     * @param radius     shared radius
     * @param isLeftTurn turn direction of circle 1
     * @return [tangent1 on circle1, tangent2 on circle2]
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
        return new Pose2d[]{
                new Pose2d(center1.plus(offset).getX(), center1.plus(offset).getY(), center1.plus(offset).getAngle()),
                new Pose2d(center2.minus(offset).getX(), center2.minus(offset).getY(), center2.minus(offset).getAngle())  // opposite sign — cross tangent
        };
    }
}
