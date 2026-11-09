package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.constants.PathConstants;

public class LineSegment extends SegmentBase {
    private Translation2d startToEndVector;
    private double finalVelocity;

    public LineSegment(double finalVelocity, Translation2d startPoint, Translation2d endPoint) {
        super(startPoint, endPoint);
        this.startToEndVector = endPoint.minus(startPoint);
        this.finalVelocity = finalVelocity;
    }

    public LineSegment(Translation2d startPoint, Translation2d endPoint) {
        this(PathConstants.MAX_VELOCITY, startPoint, endPoint);
    }

    public Translation2d getTranslation() {
        return startToEndVector;
    }

    public double getFinalVelocity() {
        return finalVelocity;
    }
}
