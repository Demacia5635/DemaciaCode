package frc.demacia.path;

/**
 * The speed limits and "close enough" distances for one path.
 * Pass it to {@link TrajectoryBuilder#build(java.util.List, PathConfig)}.
 * A new PathConfig has the default values. Each {@code withX} method changes one value
 * and returns the same object, so you can chain them:
 * {@code new PathConfig().withMaxSpeed(3.0).withMaxAccel(2.5)}.
 *
 * <p>The builder copies the values when it builds. Changing a config later
 * does not change paths that were already built.
 * Every value must be finite and greater than 0, or the build throws.
 * Angles are in radians, not degrees.
 */
public class PathConfig {
    public double maxSpeed = 4; // m/s
    public double maxAccel = 3; // m/s^2
    public double maxLateralAccel = 3; // m/s^2
    public double maxAngularSpeed = 2; // rad/s
    public double maxAngularAccel = 0.6; // rad/s^2, not used by PathFollower right now
    // "reached" distances: how close counts as being at the end of a segment
    public double maxPositionThreshold = 0.1; // m, during the path
    public double finalPositionThreshold = 0.1; // m, at a stop point
    public double passedPointAngle = Math.toRadians(60); // rad
    public double finalHeadingTolerance = Math.toRadians(5); // rad

    /** @param maxSpeed top translation speed, in m/s */
    public PathConfig withMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        return this;
    }

    /** @param maxAccel speed-up and slow-down rate, in m/s^2 */
    public PathConfig withMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    /**
     * @param maxLateralAccel sideways acceleration in corners, in m/s^2.
     *        A smaller value makes wider, slower corners.
     */
    public PathConfig withMaxLateralAccel(double maxLateralAccel) {
        this.maxLateralAccel = maxLateralAccel;
        return this;
    }

    /** @param maxAngularSpeed top turning speed, in rad/s */
    public PathConfig withMaxAngularSpeed(double maxAngularSpeed) {
        this.maxAngularSpeed = maxAngularSpeed;
        return this;
    }

    /**
     * @param maxAngularAccel turning speed-up rate, in rad/s^2.
     *        Not used by PathFollower right now: it only caps the turning speed (maxAngularSpeed).
     */
    public PathConfig withMaxAngularAccel(double maxAngularAccel) {
        this.maxAngularAccel = maxAngularAccel;
        return this;
    }

    /** @param maxPositionThreshold distance at which a line or arc end in the middle of the path counts as reached, in m */
    public PathConfig withMaxPositionThreshold(double maxPositionThreshold) {
        this.maxPositionThreshold = maxPositionThreshold;
        return this;
    }

    /** @param finalPositionThreshold distance at which a stop point or the final point counts as reached, in m */
    public PathConfig withFinalPositionThreshold(double finalPositionThreshold) {
        this.finalPositionThreshold = finalPositionThreshold;
        return this;
    }

    /**
     * @param passedPointAngle if the end of a line is more than this angle away from the
     *        line direction, the robot has passed it, in rad (not degrees)
     */
    public PathConfig withPassedPointAngle(double passedPointAngle) {
        this.passedPointAngle = passedPointAngle;
        return this;
    }

    /** @param finalHeadingTolerance allowed heading error at the end of the path, in rad (5 means 5 rad, not 5 degrees) */
    public PathConfig withFinalHeadingTolerance(double finalHeadingTolerance) {
        this.finalHeadingTolerance = finalHeadingTolerance;
        return this;
    }
}
