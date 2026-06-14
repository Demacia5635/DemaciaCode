package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.log.LogManager;

/**
 * Holds a circle used for curved path segments.
 * Stores the center, radius, and turn direction.
 */
public class CircleCalculator {
    // public final Translation2d center;
    // public final double radius;
    // public final boolean isLeftTurn;

    // public CircleCalculator(Translation2d center, double radius, boolean isLeftTurn) {
    //     this.center = center;
    //     this.radius = radius;
    //     this.isLeftTurn = isLeftTurn;
    // }

    /**
     * Calculates the circle center for a given midpoint using the bisector of the angle
     * between the vectors midPoint->from and midPoint->to.
     *
     * @param p1      previous waypoint (P1)
     * @param p3        next waypoint (P3)
     * @param p2  the waypoint the circle is built around (P2)
     * @param radius    desired turn radius
     * @return An ArcSegment object containing the start, finish, and center.
     */
    public static Circle calculateCircleCenter(Translation2d p1,
                                                   Translation2d p3,
                                                   Translation2d p2,
                                                   double radius) {

        double midPointToFromAngle = p1.minus(p2).getAngle().getRadians();
        double midPointToToAngle   = p3.minus(p2).getAngle().getRadians();

        // Angle of the bisector between the two vectors
        double vecAngle = (midPointToFromAngle + midPointToToAngle) / 2;
        if (Math.abs(midPointToFromAngle - midPointToToAngle) > Math.PI){
            vecAngle += Math.PI;
        }

        // Positive angle diff = right turn, negative = left turn
        double angleDiff = angleModulus(midPointToFromAngle + Math.PI - midPointToToAngle);
        // הערה: משתנה זה (isLeftTurn) כרגע לא נכנס ישירות ל-ArcSegment, 
        // אך הוא נשאר כאן למקרה שתצטרך אותו בעתיד.
        boolean isLeftTurn = angleDiff < 0; 
        LogManager.log(vecAngle + " " + midPointToFromAngle + " " + midPointToToAngle);

        if (isLeftTurn){
            vecAngle += Math.PI;
        }

        Translation2d center = p2.plus(
                new Translation2d(radius * Math.cos(vecAngle), radius * Math.sin(vecAngle)));

        // מכיוון ש-ArcSegment דורש Pose2d, אנו מייצרים אותם מתוך ה-Translation2d.
        // כאן השתמשתי בזווית הנוכחית של המסלול כברירת מחדל (new Rotation2d()).
        // אם יש לך זוויות כיוון מדויקות לנקודות ההתחלה והסיום, מומלץ להעביר אותן לכאן.
        // Pose2d startPose = new Pose2d(p1, new Rotation2d(midPointToFromAngle));
        // Pose2d finishPose = new Pose2d(p3, new Rotation2d(midPointToToAngle));

        return new Circle(center, radius, isLeftTurn);
    }

    /** Wraps angle to the range [-2π, 2π] */
    private static double angleModulus(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}