package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.constants.PathConstants;

public class LineSegment extends SegmentBase {
    private Translation2d startToEndVector;
    private double finalVelocity;

    public LineSegment(double finalVelocity, Translation2d startPoint, Translation2d endPoint, Rotation2d endRotation) {
        super(startPoint, new Pose2d(endPoint, endRotation));
        this.startToEndVector = endPoint.minus(startPoint);
        this.finalVelocity = finalVelocity;
    }

    public LineSegment(Translation2d startPoint, Translation2d endPoint, Rotation2d endRotation) {
        this(PathConstants.MAX_VELOCITY, startPoint, endPoint, endRotation);
    }

    public LineSegment(Translation2d startPoint, Pose2d endPoint) {
        this(PathConstants.MAX_VELOCITY, startPoint, endPoint.getTranslation(), endPoint.getRotation());
    }

    public Translation2d getTranslation() {
        return startToEndVector;
    }

    public double getFinalVelocity() {
        return finalVelocity;
    }
}
