package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.DemaciaTrapezoid;

/**
 * One piece of the route: a straight line (LineSegment) or an arc (ArcSegment).
 * TrajectoryBuilder creates the segments once. PathFollower asks the current one
 * for a velocity every cycle and asks it when it is finished.
 * This base class handles speed (with a DemaciaTrapezoid). The subclasses handle direction and distance.
 * All positions are field-relative, in meters.
 */
public abstract class BaseSegment {

    private final Translation2d startTranslation;
    private final Pose2d endPose;
    private final double endVelocity; // m/s
    private final double maxVelocity; // m/s
    private final DemaciaTrapezoid trapezoid;

    /**
     * @param startTranslation where the segment starts, in m
     * @param endPose where the segment ends, in m, plus the heading the robot should have there
     * @param endVelocity planned speed at the end, in m/s (0 means a stop point)
     * @param maxVelocity top speed on this segment, in m/s
     * @param maxAccel speed-up and slow-down rate, in m/s^2
     */
    public BaseSegment(Translation2d startTranslation, Pose2d endPose, double endVelocity, double maxVelocity, double maxAccel) {
        this.startTranslation = startTranslation;
        this.endPose = endPose;
        this.endVelocity = endVelocity;
        this.maxVelocity = maxVelocity;
        this.trapezoid = new DemaciaTrapezoid(maxVelocity, maxAccel);
    }

    /**
     * Works out the velocity to drive in this cycle.
     * The speed comes from the trapezoid, the direction from the line or arc shape.
     *
     * @param position robot position, field-relative, in m
     * @param fieldVelocity measured robot velocity, field-relative, in m/s
     * @return wanted velocity, field-relative, in m/s
     */
    public Translation2d getVelocity(Translation2d position, Translation2d fieldVelocity) {
        double currentVelocity = Math.hypot(fieldVelocity.getX(), fieldVelocity.getY());
        double speed = Math.min(trapezoid.nextVelocity(getDistanceLeft(position), currentVelocity, endVelocity), maxVelocity);
        return new Translation2d(speed, getDirection(position, currentVelocity));
    }

    /**
     * @param position robot position, field-relative, in m
     * @param currentVelocity measured speed of the robot, in m/s
     * @return the direction to drive, field-relative
     */
    public abstract Rotation2d getDirection(Translation2d position, double currentVelocity);

    /**
     * @param position robot position, field-relative, in m
     * @return how far is left to drive on this segment, in m (the trapezoid uses it to know when to brake)
     */
    public abstract double getDistanceLeft(Translation2d position);

    /**
     * @param position robot position, field-relative, in m
     * @return true when the robot is close enough to the end (or past it) to move on
     */
    public abstract boolean isFinished(Translation2d position);

    /** @return length of the segment along its path, in m */
    public abstract double getLength();

    /** @return where the segment ends (m) and the heading the robot should have there */
    public Pose2d getEndPose() {
        return endPose;
    }

    /** @return top speed on this segment, in m/s */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /** @return planned speed at the end of the segment, in m/s (0 means a stop point) */
    public double getEndVelocity() {
        return endVelocity;
    }

    @Override
    public String toString() {
        return "Start point: " + startTranslation + " Finish Point: " + endPose
                + " End velocity: " + endVelocity + " Max velocity: " + maxVelocity;
    }
}
