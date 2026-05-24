package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Translation2d;
 
public class Circle {
    public final Translation2d center;
    public final double radius;
    public final boolean isLeftTurn;
 
    public Circle(Translation2d center, double radius, boolean isLeftTurn) {
        this.center = center;
        this.radius = radius;
        this.isLeftTurn = isLeftTurn;
    }
 
    // Builds a circle around midPoint using the bisector of the angle between from and to.
    // isLeftTurn is determined by the sign of the angle difference.
    public static Circle calculate(Translation2d from, Translation2d to,
                                   Translation2d midPoint, double radius) {
        double fromAngle = from.minus(midPoint).getAngle().getRadians();
        double toAngle   = to.minus(midPoint).getAngle().getRadians();
 
        double bisector  = (fromAngle + toAngle) / 2;
        double angleDiff = angleModulus(fromAngle + Math.PI - toAngle);
 
        Translation2d center = midPoint.plus(
                new Translation2d(radius * Math.cos(bisector), radius * Math.sin(bisector)));
 
        return new Circle(center, radius, angleDiff < 0);
    }
 
    private static double angleModulus(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}