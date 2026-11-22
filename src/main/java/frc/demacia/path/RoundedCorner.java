package frc.demacia.path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.log.Log;

/**
 * The circle that rounds one corner of the path.
 * TrajectoryBuilder asks for one of these for every waypoint in the middle of the path.
 * It only holds the circle (center, radius, speed). The builder then works out the
 * straight lines that touch the circles.
 */
class RoundedCorner {
    public final Translation2d center; // the point itself when it has no arc
    public final double signedRadius; // m, positive = left arc, negative = right arc, 0 = no arc
    public final double speed; // m/s, when passing the point

    RoundedCorner(Translation2d center, double signedRadius, double speed) {
        this.center = center;
        this.signedRadius = signedRadius;
        this.speed = speed;
    }

    /**
     * Works out the corner circle at a waypoint. The circle goes through the waypoint.
     * If the corner should have no arc (a turn under 1 degree, a near U-turn over 179 degrees,
     * a speed of 0, or legs that are too short), the result has radius 0.
     * A near U-turn also gets speed 0, so the robot stops there.
     *
     * @param previous field position of the point before, in m
     * @param waypoint the corner point
     * @param next field position of the point after, in m
     * @param maxCornerSpeed corner speed limit, in m/s (already cut to the path max speed)
     * @param config the path limits (uses maxLateralAccel)
     * @return the circle: center in m, signed radius in m, and corner speed in m/s
     */
    public static RoundedCorner fromWaypoints(Translation2d previous, Waypoint waypoint, Translation2d next,
            double maxCornerSpeed, PathConfig config) {
        Translation2d point = waypoint.getTranslation();
        double speed = maxCornerSpeed;

        double turnAngle = Math.abs(MathUtil.angleModulus(
                next.minus(point).getAngle().getRadians() - point.minus(previous).getAngle().getRadians()));
        double shorterLeg = Math.min(previous.getDistance(point), point.getDistance(next));
        // The arc may use at most 1/4 of the shorter leg, so arcs of nearby corners do not overlap.
        double maxRadius = shorterLeg * PathConstants.MAX_RADIUS_TO_LEG_RATIO;

        boolean isReversal = turnAngle > Math.PI - PathConstants.MIN_TURN_FOR_ARC;
        boolean isAlmostStraight = turnAngle < PathConstants.MIN_TURN_FOR_ARC;
        boolean isTooTight = maxRadius < 1e-6;

        if (speed == 0 || isReversal || isAlmostStraight || isTooTight) {
            double speedAtPoint = isReversal ? 0 : speed;
            return new RoundedCorner(point, 0, speedAtPoint);
        }

        // Cornering needs a sideways acceleration of v^2 / r. Solve for r with the sideways
        // limit, so the faster we go, the wider the arc has to be.
        double radius = speed * speed / config.maxLateralAccel;
        if (radius > maxRadius) {
            // The arc got smaller, so the speed must drop too: v = sqrt(a * r)
            radius = maxRadius;
            speed = Math.sqrt(config.maxLateralAccel * radius);
        }

        // the center is on the bisector of the corner, radius away from the point
        double angleToPrevious = previous.minus(point).getAngle().getRadians();
        double angleToNext = next.minus(point).getAngle().getRadians();
        double centerAngle = (angleToPrevious + angleToNext) / 2;
        if (Math.abs(angleToPrevious - angleToNext) > Math.PI) {
            centerAngle += Math.PI;
        }
        double angleDiff = MathUtil.angleModulus(angleToNext - angleToPrevious);
        boolean isLeftTurn = angleDiff < 0;
        Translation2d center = point.plus(new Translation2d(radius * Math.cos(centerAngle), radius * Math.sin(centerAngle)));
        Log.log("angleDiff: " + angleDiff + " from: " + previous + " midPoint: "  + point + " to: " + next + " vecAngle: " + centerAngle + " center: " + center + " isLeftTurn: " + isLeftTurn + " MidPointToFromAngle: " + angleToPrevious + " MidToPointToAngle: " + angleToNext);

        double signedRadius = isLeftTurn ? radius : -radius;
        return new RoundedCorner(center, signedRadius, speed);
    }
}
