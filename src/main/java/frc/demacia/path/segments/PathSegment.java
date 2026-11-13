package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A single piece of a trajectory: it starts at a point and ends at a pose. */
public abstract class PathSegment {

    private final Translation2d startTranslation;
    private final Pose2d endPose;
    private final double endVelocityMps;
    private final double maxVelocityMps;

    protected PathSegment(Translation2d startTranslation, Pose2d endPose, double endVelocityMps, double maxVelocityMps) {
        this.startTranslation = startTranslation;
        this.endPose = endPose;
        this.endVelocityMps = endVelocityMps;
        this.maxVelocityMps = maxVelocityMps;
    }

    public Translation2d getStartTranslation() {
        return startTranslation;
    }

    public Pose2d getEndPose() {
        return endPose;
    }

    public double getEndVelocityMps() {
        return endVelocityMps;
    }

    public double getMaxVelocityMps() {
        return maxVelocityMps;
    }

    @Override
    public String toString() {
        return "Start point: " + startTranslation + " End point: " + endPose;
    }
}
