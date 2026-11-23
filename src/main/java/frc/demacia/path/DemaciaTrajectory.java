package frc.demacia.path;

import java.util.List;

import frc.demacia.path.segments.BaseSegment;

/**
 * The finished route: an ordered list of lines and arcs, plus the heading limits.
 * TrajectoryBuilder makes it once. PathFollower reads it every 20 ms.
 * It never changes after it is built, so many commands can share one.
 */
public class DemaciaTrajectory {
    private final List<BaseSegment> segments;
    private final double maxAngularSpeed; // rad/s
    private final double maxAngularAccel; // rad/s^2
    private final double finalHeadingTolerance; // rad

    /**
     * Normally you do not call this. Use {@link TrajectoryBuilder#build} instead.
     *
     * @param segments the lines and arcs in driving order (the list is copied)
     * @param maxAngularSpeed fastest allowed turning speed, in rad/s
     * @param maxAngularAccel fastest allowed change of turning speed, in rad/s^2 (not used by PathFollower right now)
     * @param finalHeadingTolerance how far the final heading may be off, in rad
     */
    public DemaciaTrajectory(List<BaseSegment> segments, double maxAngularSpeed,
            double maxAngularAccel, double finalHeadingTolerance) {
        // copy the list so nobody can change the trajectory after it is built
        this.segments = List.copyOf(segments);
        this.maxAngularSpeed = maxAngularSpeed;
        this.maxAngularAccel = maxAngularAccel;
        this.finalHeadingTolerance = finalHeadingTolerance;
    }

    /**
     * @return the lines and arcs in driving order (cannot be changed)
     */
    public List<BaseSegment> getSegments() {
        return segments;
    }

    /**
     * @return fastest allowed turning speed, in rad/s
     */
    public double getMaxAngularSpeed() {
        return maxAngularSpeed;
    }

    /**
     * @return fastest allowed change of turning speed, in rad/s^2 (not used by PathFollower right now)
     */
    public double getMaxAngularAccel() {
        return maxAngularAccel;
    }

    /**
     * @return allowed error of the final heading, in rad
     */
    public double getFinalHeadingTolerance() {
        return finalHeadingTolerance;
    }
}
