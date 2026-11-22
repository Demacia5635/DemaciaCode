package frc.demacia.path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.PathConfig;

/**
 * A straight piece of the route.
 * It steers the robot back onto the line if the robot drifts off it.
 */
public class LineSegment extends PathSegment {
    private final Translation2d startToEndVector;
    private final double maxPositionThreshold; // m
    private final double finalPositionThreshold; // m
    private final double passedPointAngle; // rad

    /**
     * The distances and angle are copied from the config now, so changing the config later has no effect.
     *
     * @param startPoint where the line starts, in m
     * @param endPoint where the line ends (m), plus the heading to have there
     * @param endVelocity planned speed at the end, in m/s (0 means a stop point)
     * @param maxVelocity top speed on the line, in m/s
     * @param config path limits (acceleration and the "reached" distances and angle)
     */
    public LineSegment(Translation2d startPoint, Pose2d endPoint, double endVelocity, double maxVelocity, PathConfig config) {
        super(startPoint, endPoint, endVelocity, maxVelocity, config.maxAccel);
        this.startToEndVector = endPoint.getTranslation().minus(startPoint);
        this.maxPositionThreshold = config.maxPositionThreshold;
        this.finalPositionThreshold = config.finalPositionThreshold;
        this.passedPointAngle = config.passedPointAngle;
    }

    /**
     * @param position robot position, field-relative, in m
     * @param currentVelocity measured speed, in m/s (not used by a line)
     * @return the direction to drive, field-relative
     */
    @Override
    public Rotation2d getDirection(Translation2d position, double currentVelocity) {
        Translation2d toEnd = getEndPose().getTranslation().minus(position);
        // the robot is on the end point: toEnd has no direction, so use the line direction
        if (toEnd.getNorm() < 1e-6) {
            return startToEndVector.getAngle();
        }
        // passed the end: aim straight at it
        if (angleBetweenLineAndEnd(toEnd) > Math.PI / 2) {
            return toEnd.getAngle();
        }
        // "Mirroring": take the line direction and mirror it around the direction to the end.
        // If the end is e radians off the line direction, we drive e further than the end,
        // which is 2e off the line. This over-steers on purpose, so the robot comes back
        // onto the line instead of cutting straight to the end.
        return new Rotation2d(2 * toEnd.getAngle().getRadians() - startToEndVector.getAngle().getRadians());
    }

    /**
     * @param position robot position, field-relative, in m
     * @return straight-line distance to the end point, in m
     */
    @Override
    public double getDistanceLeft(Translation2d position) {
        return getEndPose().getTranslation().minus(position).getNorm();
    }

    /**
     * A stop point or the final point is finished only when the robot is within finalPositionThreshold.
     * Any other line is finished when the robot is within maxPositionThreshold of the end,
     * or has driven past the end.
     *
     * @param position robot position, field-relative, in m
     * @return true if the line is finished
     */
    @Override
    public boolean isFinished(Translation2d position) {
        Translation2d toEnd = getEndPose().getTranslation().minus(position);
        double distance = toEnd.getNorm();
        if (getEndVelocity() == 0) {
            return distance < finalPositionThreshold;
        }
        // "passed": the end is now behind the robot (more than passedPointAngle off the line direction)
        boolean passed = distance > 1e-6 && angleBetweenLineAndEnd(toEnd) > passedPointAngle;
        return distance < maxPositionThreshold || passed;
    }

    // rad, how far the direction to the end is from the direction of the line, always positive
    private double angleBetweenLineAndEnd(Translation2d toEnd) {
        double angleDifference = toEnd.getAngle().getRadians() - startToEndVector.getAngle().getRadians();
        return Math.abs(MathUtil.angleModulus(angleDifference));
    }

    /** @return length of the line, in m */
    @Override
    public double getLength() {
        return startToEndVector.getNorm();
    }
}
