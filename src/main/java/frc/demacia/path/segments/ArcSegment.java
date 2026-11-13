package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.constants.PathConstants;

/** A circular-arc piece of the trajectory (used to round the corners between lines). */
public class ArcSegment extends PathSegment {

    private final Translation2d circleCenter;
    private final Translation2d centerToStart;
    private final Translation2d centerToEnd;
    private final boolean isLeftTurn;
    private final double radiusMeters;

    public ArcSegment(Translation2d startTranslation, Pose2d endPose, Translation2d circleCenter,
            boolean isLeftTurn, double radiusMeters, double endVelocityMps, double maxVelocityMps) {
        super(startTranslation, endPose, endVelocityMps, maxVelocityMps);
        this.circleCenter = circleCenter;
        this.isLeftTurn = isLeftTurn;
        this.radiusMeters = radiusMeters;
        this.centerToStart = startTranslation.minus(circleCenter);
        this.centerToEnd = endPose.getTranslation().minus(circleCenter);
    }

    public ArcSegment(Translation2d startTranslation, Pose2d endPose, Translation2d circleCenter,
            boolean isLeftTurn, double radiusMeters) {
        this(startTranslation, endPose, circleCenter, isLeftTurn, radiusMeters,
                PathConstants.MAX_VELOCITY_MPS, PathConstants.MAX_VELOCITY_MPS);
    }

    public Translation2d getCenter() {
        return circleCenter;
    }

    public boolean isLeftTurn() {
        return isLeftTurn;
    }

    public double getRadiusMeters() {
        return radiusMeters;
    }

    /** Angle between the center-to-start and center-to-end vectors. */
    public Rotation2d getSweepAngle() {
        return centerToEnd.getAngle().minus(centerToStart.getAngle());
    }

    /** Angle of the center-to-end vector. */
    public Rotation2d getEndAngleFromCenter() {
        return centerToEnd.getAngle();
    }

    @Override
    public String toString() {
        return super.toString() + " Circle center: " + circleCenter;
    }
}
