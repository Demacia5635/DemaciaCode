package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Holds a circle used for curved path segments.
 * Stores the center, radius, and turn direction.
 */
public class CircleCalculator {
    public final Translation2d center;
    public final double radius;
    public final boolean isLeftTurn;

    public CircleCalculator(Translation2d center, double radius, boolean isLeftTurn) {
        this.center = center;
        this.radius = radius;
        this.isLeftTurn = isLeftTurn;
    }

    /**
     * Calculates the circle center for a given midpoint using the bisector of the angle
     * between the vectors midPoint->from and midPoint->to.
     *
     * @param from      previous waypoint (P1)
     * @param to        next waypoint (P3)
     * @param midPoint  the waypoint the circle is built around (P2)
     * @param radius    desired turn radius
     */
    public static CircleCalculator calculateCircleCenter(Translation2d from,
                                               Translation2d to,
                                               Translation2d midPoint,
                                               double radius) {
        double midPointToFromAngle = from.minus(midPoint).getAngle().getRadians();
        double midPointToToAngle   = to.minus(midPoint).getAngle().getRadians();

        // Angle of the bisector between the two vectors
        double vecAngle = (midPointToFromAngle + midPointToToAngle) / 2;

        // Positive angle diff = right turn, negative = left turn
        double angleDiff = angleModulus(midPointToFromAngle + Math.PI - midPointToToAngle);
        boolean isLeftTurn = angleDiff < 0;

        Translation2d center = midPoint.plus(
                new Translation2d(radius * Math.cos(vecAngle), radius * Math.sin(vecAngle)));

        return new CircleCalculator(center, radius, isLeftTurn);
    }

    /** Wraps angle to the range [-π, π] */
    private static double angleModulus(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    }
