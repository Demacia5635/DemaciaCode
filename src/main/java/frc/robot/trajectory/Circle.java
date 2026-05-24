package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
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