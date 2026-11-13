package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.constants.PathConstants;

/** A straight piece of the trajectory. */
public class LineSegment extends PathSegment {

    private final Translation2d startToEndVector;

    public LineSegment(Translation2d startTranslation, Pose2d endPose, double endVelocityMps, double maxVelocityMps) {
        super(startTranslation, endPose, endVelocityMps, maxVelocityMps);
        this.startToEndVector = endPose.getTranslation().minus(startTranslation);
    }

    public LineSegment(Translation2d startTranslation, Pose2d endPose) {
        this(startTranslation, endPose, PathConstants.MAX_VELOCITY_MPS, PathConstants.MAX_VELOCITY_MPS);
    }

    public Translation2d getStartToEndVector() {
        return startToEndVector;
    }
}
