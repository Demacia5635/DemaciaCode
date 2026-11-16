package frc.demacia.path.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.log.Log;

/** Rounds one corner of the path with a circular arc. */
public final class CornerRounder {

    private CornerRounder() {}

    /**
     * Finds the circle that rounds the corner, and where the robot enters and leaves it.
     *
     * @param previousPoint the waypoint before the corner
     * @param cornerPose    the waypoint being rounded (its rotation is the heading on the arc)
     * @param nextPoint     the waypoint after the corner
     * @param radiusMeters  radius of the rounding circle
     */
    public static RoundedCorner round(Translation2d previousPoint, Pose2d cornerPose,
            Translation2d nextPoint, double radiusMeters) {
        Translation2d cornerPoint = cornerPose.getTranslation();

        // Step 1: circle center on the corner bisector, one radius away from the corner
        double cornerToPreviousAngleRad = previousPoint.minus(cornerPoint).getAngle().getRadians();
        double cornerToNextAngleRad = nextPoint.minus(cornerPoint).getAngle().getRadians();

        double bisectorAngleRad = (cornerToPreviousAngleRad + cornerToNextAngleRad) / 2;
        if (Math.abs(cornerToPreviousAngleRad - cornerToNextAngleRad) > Math.PI) {
            bisectorAngleRad += Math.PI;
        }

        double turnAngleRad = MathUtil.angleModulus(cornerToNextAngleRad - cornerToPreviousAngleRad);
        boolean isLeftTurn = turnAngleRad < 0;

        Translation2d center = cornerPoint.plus(
                new Translation2d(radiusMeters * Math.cos(bisectorAngleRad), radiusMeters * Math.sin(bisectorAngleRad)));

        Log.log("turnAngle: " + turnAngleRad + " previous: " + previousPoint + " corner: " + cornerPose
                + " next: " + nextPoint + " bisectorAngle: " + bisectorAngleRad + " center: " + center
                + " isLeftTurn: " + isLeftTurn + " cornerToPreviousAngle: " + cornerToPreviousAngleRad
                + " cornerToNextAngle: " + cornerToNextAngleRad);

        // Step 2: entry point = tangent from the previous waypoint to the circle
        Translation2d previousToCenter = center.minus(previousPoint);
        double distanceToCenter = previousToCenter.getNorm();

        // Angle at the external point between the line to the center and the tangent line
        double tangentAngleRad = Math.asin(radiusMeters / distanceToCenter);

        Translation2d tangentDirection = previousToCenter.rotateBy(
                new Rotation2d(isLeftTurn ? -tangentAngleRad : tangentAngleRad));
        double tangentLength = Math.cos(tangentAngleRad) * distanceToCenter;
        Translation2d previousToEntry = tangentDirection.div(tangentDirection.getNorm()).times(tangentLength);

        Translation2d entryPoint = previousPoint.plus(previousToEntry);

        // Step 3: exit point = entry point mirrored across the center-to-corner line
        Translation2d centerToEntry = entryPoint.minus(center);
        Translation2d centerToCorner = cornerPoint.minus(center);
        Translation2d exitPoint = centerToCorner
                .rotateBy(centerToCorner.getAngle().minus(centerToEntry.getAngle()))
                .plus(center);

        Rotation2d heading = cornerPose.getRotation();
        return new RoundedCorner(new Pose2d(entryPoint, heading), new Pose2d(exitPoint, heading), center, isLeftTurn);
    }
}
