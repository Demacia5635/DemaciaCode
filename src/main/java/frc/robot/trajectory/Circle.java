package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Holds a circle used for curved path segments.
 * Stores the center, radius, and turn direction.
 */
public class Circle {
    public final Translation2d center;
    public final double radius;
    public final boolean isLeftTurn;

    public Circle(Translation2d center, double radius, boolean isLeftTurn) {
        this.center = center;
        this.radius = radius;
        this.isLeftTurn = isLeftTurn;
    }

<<<<<<< HEAD
    private double getDeltaAngle(Translation2d start, Translation2d end) {
        double startAngle = Math.atan2(start.getY() - center.getY(), start.getX() - center.getX());
        double endAngle = Math.atan2(end.getY() - center.getY(), end.getX() - center.getX());
        double deltaAngle = endAngle - startAngle;
        if (isLeftTurn && deltaAngle < 0) {
            deltaAngle += 2 * Math.PI;
        } else if (!isLeftTurn && deltaAngle > 0) {
            deltaAngle -= 2 * Math.PI;
        }
        return deltaAngle;
    }

    public double getArchDistance() {
        return radius * getDeltaAngle(new Translation2d(), new Translation2d());
    }

    public double getHeading(Pose2d p1, Pose2d p2, double angle) {
        Translation2d v = p1.getTranslation().plus(p2.getTranslation());
        Translation2d Vector = new Translation2d(v.getAngle().getRadians() + getArchDistance(),Math.sqrt(-1));
        return 2* angle +Vector.getAngle().getRadians();
    }

    public double getVelocity(double maxVelocity, double maxAcceleration, double distanceLeft, double currentVelocity, double finishVelocity) {
        DemaciaTrapezoid trapezoid = new DemaciaTrapezoid(maxVelocity, maxAcceleration);
        return trapezoid.calculate(distanceLeft, currentVelocity, finishVelocity);
    }
 
}
=======
    /**
     * Calculates the circle center for a given midpoint using the bisector of the angle
     * between the vectors midPoint->from and midPoint->to.
     *
     * @param from      previous waypoint (P1)
     * @param to        next waypoint (P3)
     * @param midPoint  the waypoint the circle is built around (P2)
     * @param radius    desired turn radius
     */
    public static Circle calculateCircleCenter(Translation2d from,
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

        return new Circle(center, radius, isLeftTurn);
    }

    /** Wraps angle to the range [-π, π] */
    private static double angleModulus(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
>>>>>>> parent of 001daa0 (change after matan saw it)
