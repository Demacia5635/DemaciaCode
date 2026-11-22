package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * One point on the route: a field position plus the heading the robot should have there.
 * You give TrajectoryBuilder a list of these.
 * The rotation is the heading for the segments that end at this point.
 *
 * <p>A waypoint is a Pose2d with one extra number, the corner speed limit.
 * {@code withMaxCornerSpeed(0)} makes the robot stop exactly at this point.
 *
 * <pre>{@code
 * List<Waypoint> points = List.of(
 *         new Waypoint(startPose),
 *         new Waypoint(pickupPose).withMaxCornerSpeed(2.0), // slow down to 2 m/s at the corner
 *         new Waypoint(scorePose));
 * }</pre>
 */
public class Waypoint extends Pose2d {
    private final double maxCornerSpeed; // m/s, infinite = path max speed

    /**
     * @param pose field pose (meters, rotation); the corner speed is not limited
     */
    public Waypoint(Pose2d pose) {
        this(pose.getTranslation(), pose.getRotation());
    }

    /**
     * @param translation field position, in m
     * @param heading wanted robot heading, field-relative
     */
    public Waypoint(Translation2d translation, Rotation2d heading) {
        this(translation, heading, Double.POSITIVE_INFINITY);
    }

    /**
     * @param x field x, in m
     * @param y field y, in m
     * @param heading wanted robot heading, field-relative
     */
    public Waypoint(double x, double y, Rotation2d heading) {
        this(new Translation2d(x, y), heading);
    }

    private Waypoint(Translation2d translation, Rotation2d heading, double maxCornerSpeed) {
        super(translation, heading);
        this.maxCornerSpeed = maxCornerSpeed;
    }

    /**
     * Makes a copy of this waypoint with a corner speed limit. This waypoint does not change.
     * 0 means stop here. A speed above the path's max speed is cut down to the max speed.
     * TrajectoryBuilder.build throws if the value is negative or NaN,
     * or if it is set on the first or last waypoint.
     *
     * @param mps the most the robot may drive through this point, in m/s
     * @return the new waypoint
     */
    public Waypoint withMaxCornerSpeed(double mps) {
        return new Waypoint(getTranslation(), getRotation(), mps);
    }

    public double getMaxCornerSpeed() {
        return maxCornerSpeed;
    }
}
