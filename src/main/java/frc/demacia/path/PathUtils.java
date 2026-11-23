package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Small math helpers used by the path classes. They only take inputs and return a result,
 * they don't remember anything.
 */
public class PathUtils {

    /**
     * NaN guard: one bad sensor value would otherwise poison the turning and the segment math.
     *
     * @return false if the pose or the speeds contain NaN or infinity
     */
    public static boolean allNumbersAreFinite(Pose2d pose, ChassisSpeeds speeds) {
        double heading = pose.getRotation().getRadians();
        return Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(heading)
                && Double.isFinite(speeds.vxMetersPerSecond)
                && Double.isFinite(speeds.vyMetersPerSecond)
                && Double.isFinite(speeds.omegaRadiansPerSecond);
    }

    /**
     * How far around the circle an arc goes.
     *
     * @param start arc start, in m
     * @param end arc end, in m
     * @param center circle center, in m
     * @param isLeft true for a left (counter-clockwise) turn
     * @return the angle from start to end in the turn direction, in rad, above 0 and up to 2 pi
     *         (start == end gives 2 pi, a full circle, because MathUtil.inputModulus never returns 0 here;
     *         the builder never makes such an arc, corners under MIN_TURN_FOR_ARC get no arc)
     */
    public static double arcSweep(Translation2d start, Translation2d end, Translation2d center, boolean isLeft) {
        double angle = end.minus(center).getAngle().getRadians() - start.minus(center).getAngle().getRadians();
        return MathUtil.inputModulus(isLeft ? angle : -angle, 0, 2 * Math.PI);
    }

    /**
     * Finds the straight line that touches circle 1 and circle 2 (an "outer or inner tangent").
     * A plain point is treated as a circle with radius 0.
     *
     * @param signedRadius1 positive = left (counter-clockwise) arc, negative = right arc, 0 = a plain point
     * @param signedRadius2 same as signedRadius1, for circle 2
     * @return the points where the straight leg leaves circle 1 and touches circle 2,
     *         or an empty list when there is no such leg (the circles are too close)
     */
    public static List<Translation2d> tangentPoints(Translation2d center1, double signedRadius1, Translation2d center2, double signedRadius2) {
        List<Translation2d> points = new ArrayList<>();
        Translation2d centerToCenter = center2.minus(center1);

        if (centerToCenter.getNorm() < 1e-9) {
            points.add(center1);
            points.add(center2);
            return points;
        }
        // If the radii differ by at least the distance between the centers, one circle is
        // inside the other and no line can touch both.
        if (Math.abs(signedRadius2 - signedRadius1) >= centerToCenter.getNorm()) {
            return points;
        }

        // the angle between the line connecting the centers and the leg
        double alpha = Math.asin((signedRadius2 - signedRadius1) / centerToCenter.getNorm());

        // unit vector pointing to the left of the leg
        Translation2d left = new Translation2d(1, new Rotation2d(centerToCenter.getAngle().getRadians() - alpha + Math.PI / 2));

        points.add(center1.minus(left.times(signedRadius1)));
        points.add(center2.minus(left.times(signedRadius2)));
        return points;
    }
}
