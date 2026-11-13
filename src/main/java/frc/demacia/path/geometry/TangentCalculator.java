package frc.demacia.path.geometry;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;

/**
 * Static helpers for computing tangent points between points and circles.
 * <ol>
 *   <li>Corner arc: entry and exit points of the arc that rounds a corner (used by the trajectory builder).</li>
 *   <li>Point to circle: tangent from an external point to a circle.</li>
 *   <li>Circle to circle, same turn direction (parallel tangent).</li>
 *   <li>Circle to circle, opposite turn direction (cross tangent).</li>
 * </ol>
 */
public final class TangentCalculator {

    private TangentCalculator() {}

    /**
     * Calculates where the robot enters and leaves the arc that rounds a corner.
     * The entry point is the tangent from {@code previousPoint} to the circle.
     * The exit point is the mirror image of the entry point across the center-to-corner line
     * (the center is exactly one radius away from the corner, so both points lie on the circle).
     *
     * @param previousPoint start of the line that leads into the corner
     * @param cornerPoint   the corner waypoint
     * @param cornerArc     arc that rounds the corner (provides center, turn direction and end rotation)
     * @param radiusMeters  circle radius
     * @return [entry pose, exit pose]
     */
    public static List<Pose2d> calculateArcEntryAndExitPoses(Translation2d previousPoint, Translation2d cornerPoint,
            ArcSegment cornerArc, double radiusMeters) {
        Translation2d center = cornerArc.getCenter();
        Translation2d previousToCenter = center.minus(previousPoint);
        double distanceToCenter = previousToCenter.getNorm();

        // Angle at the external point between the line to the center and the tangent line
        double tangentAngleRad = Math.asin(radiusMeters / distanceToCenter);

        Translation2d tangentDirection = previousToCenter.rotateBy(
                new Rotation2d(cornerArc.isLeftTurn() ? -tangentAngleRad : tangentAngleRad));
        double tangentLength = Math.cos(tangentAngleRad) * distanceToCenter;
        Translation2d previousToEntry = tangentDirection.div(tangentDirection.getNorm()).times(tangentLength);

        Translation2d entryPoint = previousPoint.plus(previousToEntry);

        Translation2d centerToEntry = entryPoint.minus(center);
        Translation2d centerToCorner = cornerPoint.minus(center);
        Translation2d exitPoint = centerToCorner
                .rotateBy(centerToCorner.getAngle().minus(centerToEntry.getAngle()))
                .plus(center);

        Rotation2d heading = cornerArc.getEndPose().getRotation();
        return List.of(new Pose2d(entryPoint, heading), new Pose2d(exitPoint, heading));
    }

    /**
     * Tangent point from an external point to a circle.
     * The tangent is perpendicular to the radius, so the angle at the center is acos(r / d).
     * The sign of that angle depends on the turn direction.
     *
     * @param externalPose the external point
     * @param circleCenter circle center
     * @param radiusMeters circle radius
     * @param isLeftTurn   turn direction of the circle
     * @return the tangent point on the circle
     */
    public static Translation2d calculatePointToCircleTangent(Pose2d externalPose, Translation2d circleCenter,
            double radiusMeters, boolean isLeftTurn) {
        Translation2d centerToExternal = externalPose.getTranslation().minus(circleCenter);
        double distance = centerToExternal.getNorm();
        double baseAngleRad = centerToExternal.getAngle().getRadians();
        double tangentAngleRad = Math.acos(radiusMeters / distance);

        // Subtract for left turn, add for right turn
        double angleRad = baseAngleRad + (isLeftTurn ? -tangentAngleRad : tangentAngleRad);

        return circleCenter.plus(new Translation2d(radiusMeters * Math.cos(angleRad), radiusMeters * Math.sin(angleRad)));
    }

    /**
     * Tangent points between two circles with the SAME turn direction.
     * The tangent line is perpendicular to the center-to-center vector,
     * offset by the radius on the same side of both circles.
     *
     * @return [tangent pose on circle 1, tangent pose on circle 2]
     */
    public static Pose2d[] calculateSameTurnTangents(Translation2d center1, Translation2d center2,
            double radiusMeters, boolean isLeftTurn) {
        Translation2d center1ToCenter2 = center2.minus(center1);
        double baseAngleRad = center1ToCenter2.getAngle().getRadians();

        double offsetAngleRad = baseAngleRad + (isLeftTurn ? -Math.PI / 2 : Math.PI / 2);
        Translation2d offset = new Translation2d(radiusMeters * Math.cos(offsetAngleRad),
                radiusMeters * Math.sin(offsetAngleRad));

        Translation2d tangentPoint1 = center1.plus(offset);
        Translation2d tangentPoint2 = center2.plus(offset);

        Rotation2d drivingHeading = tangentPoint2.minus(tangentPoint1).getAngle();
        return new Pose2d[] { new Pose2d(tangentPoint1, drivingHeading), new Pose2d(tangentPoint2, drivingHeading) };
    }

    /**
     * Tangent points between two circles with OPPOSITE turn directions (cross tangent).
     * The offset angle is acos(2r / d); the offset is added to center 1 and subtracted from center 2.
     *
     * @param isLeftTurn turn direction of circle 1
     * @return [tangent pose on circle 1, tangent pose on circle 2]
     */
    public static Pose2d[] calculateOppositeTurnTangents(Translation2d center1, Translation2d center2,
            double radiusMeters, boolean isLeftTurn) {
        Translation2d center1ToCenter2 = center2.minus(center1);
        double distance = center1ToCenter2.getNorm();
        double baseAngleRad = center1ToCenter2.getAngle().getRadians();
        double crossAngleRad = Math.acos(radiusMeters * 2 / distance);

        // Add for left-to-right transition, subtract for right-to-left
        double offsetAngleRad = baseAngleRad + (isLeftTurn ? crossAngleRad : -crossAngleRad);
        Translation2d offset = new Translation2d(radiusMeters * Math.cos(offsetAngleRad),
                radiusMeters * Math.sin(offsetAngleRad));

        Translation2d tangentPoint1 = center1.plus(offset);
        Translation2d tangentPoint2 = center2.minus(offset);

        Rotation2d drivingHeading = tangentPoint2.minus(tangentPoint1).getAngle();
        return new Pose2d[] { new Pose2d(tangentPoint1, drivingHeading), new Pose2d(tangentPoint2, drivingHeading) };
    }
}
