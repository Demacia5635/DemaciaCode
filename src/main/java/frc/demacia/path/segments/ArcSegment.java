package frc.demacia.path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.PathConfig;
import frc.demacia.path.PathConstants;
import frc.demacia.path.PathUtils;

/**
 * A curved piece of the route that rounds a corner. It is part of a circle.
 * TrajectoryBuilder places it between two lines. It drives the robot along the
 * circle at the corner speed.
 */
public class ArcSegment extends BaseSegment {

    private final Translation2d center;
    private final boolean isLeft; // left = counter-clockwise
    private final double radius; // m
    private final double startAngle; // rad, center to start
    private final double sweep; // rad, start to end in the turn direction, (0, 2PI]
    private final double maxPositionThreshold; // m

    /**
     * The finish distance is copied from the config now, so changing the config later has no effect.
     *
     * @param start where the arc starts, in m
     * @param end where the arc ends (m), plus the heading to have there
     * @param center center of the circle, in m
     * @param isLeft true for a left (counter-clockwise) turn, false for a right (clockwise) turn
     * @param radius circle radius, in m (positive)
     * @param endVelocity planned speed at the end, in m/s
     * @param maxVelocity top speed on the arc, in m/s (the corner speed)
     * @param config path limits (acceleration and the "reached" distance)
     */
    public ArcSegment(Translation2d start, Pose2d end, Translation2d center, boolean isLeft, double radius,
            double endVelocity, double maxVelocity, PathConfig config) {
        super(start, end, endVelocity, maxVelocity, config.maxAccel);
        this.center = center;
        this.isLeft = isLeft;
        this.radius = radius;
        this.startAngle = start.minus(center).getAngle().getRadians();
        this.sweep = PathUtils.arcSweep(start, end.getTranslation(), center, isLeft);
        this.maxPositionThreshold = config.maxPositionThreshold;
    }

    /** @return circle radius, in m */
    public double getRadius() {
        return radius;
    }

    // flips the sign of the angle for a right (clockwise) arc
    private double applyTurnSign(double angle) {
        return isLeft ? angle : -angle;
    }

    // angle left to turn from position to the end, positive = still before the end
    private double remainingAngle(Translation2d position) {
        double robotFromStart = applyTurnSign(position.minus(center).getAngle().getRadians() - startAngle);
        // Wrap the angle around the middle of the arc (not around 0). Then an arc of more than
        // 180 degrees is measured correctly and does not jump by 2 pi.
        double travelled = MathUtil.angleModulus(robotFromStart - sweep / 2) + sweep / 2;
        return sweep - travelled;
    }

    /**
     * The direction is along the circle (the tangent), plus a small correction:
     * <ul>
     * <li>look-ahead: turn a little more, by the angle we travel in one 20 ms cycle;</li>
     * <li>pull-back: if the robot is off the circle, turn extra to get back on it.</li>
     * </ul>
     *
     * @param position robot position, field-relative, in m
     * @param currentVelocity measured speed, in m/s (used for the look-ahead)
     * @return the direction to drive, field-relative
     */
    @Override
    public Rotation2d getDirection(Translation2d position, double currentVelocity) {
        Translation2d centerToChassis = position.minus(center);
        double quarterTurn = isLeft ? Math.PI / 2 : -Math.PI / 2;
        double tangent = centerToChassis.getAngle().getRadians() + quarterTurn;
        // Look-ahead: speed / radius is the turn rate in rad/s. Times one cycle (0.02 s) it is
        // the angle we turn before the next update. We aim that far ahead so the robot does not lag behind the arc.
        double lookAhead = applyTurnSign((currentVelocity / radius) * PathConstants.DT);

        Translation2d toEnd = getEndPose().getTranslation().minus(position);
        double remaining = remainingAngle(position);
        // at or past the end: no pull-back to calculate, just follow the circle
        if (toEnd.getNorm() < 1e-6 || remaining <= 0) {
            return new Rotation2d(tangent + lookAhead);
        }
        // a robot on the arc sees the end half the remaining angle inside the tangent
        double expectedAngleToEnd = tangent + applyTurnSign(remaining / 2);
        double angleError = MathUtil.angleModulus(toEnd.getAngle().getRadians() - expectedAngleToEnd);
        // far off the arc: aim straight at the end
        if (Math.abs(angleError) > Math.PI / 2) {
            return toEnd.getAngle();
        }
        // same as the line: turn twice the error to pull the robot back onto the arc
        return new Rotation2d(tangent + lookAhead + 2 * angleError);
    }

    /**
     * @param position robot position, field-relative, in m
     * @return distance left along the arc, in m (0 if past the end)
     */
    @Override
    public double getDistanceLeft(Translation2d position) {
        return Math.max(0, remainingAngle(position)) * radius;
    }

    /**
     * @param position robot position, field-relative, in m
     * @return true when the robot has gone all the way around the arc,
     *         or is within maxPositionThreshold of the arc end
     */
    @Override
    public boolean isFinished(Translation2d position) {
        return remainingAngle(position) <= 0
                || position.getDistance(getEndPose().getTranslation()) < maxPositionThreshold;
    }

    /** @return length of the arc, in m */
    @Override
    public double getLength() {
        return radius * sweep;
    }

    @Override
    public String toString() {
        return super.toString() + " Center Circle: " + center + " Radius: " + radius + " Sweep: " + sweep;
    }
}
