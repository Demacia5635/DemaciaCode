package frc.demacia.path.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.utils.log.Log;

/** Computes the arc that rounds a corner between two consecutive lines of the path. */
public final class CornerArcCalculator {

    private CornerArcCalculator() {}

    /**
     * Finds the circle that rounds the corner at {@code cornerPoint}.
     * The circle center lies on the bisector of the corner, {@code radiusMeters} away from it.
     *
     * <p>The returned segment is only used for its center and turn direction;
     * its start/end are the neighbouring waypoints, not the real tangent points.
     *
     * @param previousPoint the waypoint before the corner
     * @param cornerPoint   the waypoint being rounded
     * @param nextPose      the waypoint after the corner
     * @param radiusMeters  radius of the rounding circle
     */
    public static ArcSegment calculateCornerArc(Translation2d previousPoint, Pose2d cornerPoint,
            Translation2d nextPose, double radiusMeters) {
        double cornerToPreviousAngleRad = previousPoint.minus(cornerPoint.getTranslation()).getAngle().getRadians();
        double cornerToNextAngleRad = nextPose.minus(cornerPoint.getTranslation()).getAngle().getRadians();

        double bisectorAngleRad = (cornerToPreviousAngleRad + cornerToNextAngleRad) / 2;
        if (Math.abs(cornerToPreviousAngleRad - cornerToNextAngleRad) > Math.PI) {
            bisectorAngleRad += Math.PI;
        }

        double turnAngleRad = MathUtil.angleModulus(cornerToNextAngleRad - cornerToPreviousAngleRad);
        boolean isLeftTurn = turnAngleRad < 0;

        Translation2d center = cornerPoint.getTranslation().plus(
                new Translation2d(radiusMeters * Math.cos(bisectorAngleRad), radiusMeters * Math.sin(bisectorAngleRad)));

        Log.log("turnAngle: " + turnAngleRad + " previous: " + previousPoint + " corner: " + cornerPoint
                + " next: " + nextPose + " bisectorAngle: " + bisectorAngleRad + " center: " + center
                + " isLeftTurn: " + isLeftTurn + " cornerToPreviousAngle: " + cornerToPreviousAngleRad
                + " cornerToNextAngle: " + cornerToNextAngleRad);

        return new ArcSegment(previousPoint, new Pose2d(nextPose, cornerPoint.getRotation()), center, isLeftTurn, radiusMeters);
    }
}
