package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.constans.PathConstants;

public class LineSegment extends SegmantBase {
    private Translation2d startToEndVector;
    private double FinalVelocity;
    // public boolean isRight;

    public LineSegment(double FinalVelocity, Translation2d startPoint, Translation2d endPoint/*, boolean isRight*/) {
        super(startPoint, endPoint);
        // this.isRight = isRight;
        this.startToEndVector = endPoint.minus(startPoint);
        this.FinalVelocity = FinalVelocity;
    }

    public LineSegment(Translation2d startPoint, Translation2d endPoint/*, boolean isRight*/) {
        this(PathConstants.MAX_VELOCITY, startPoint, endPoint/*, isRight*/);
    }

    public Translation2d getTranslation() {
        return startToEndVector;
    }

    public double getFinalVelocity() {
        return FinalVelocity;
    }
}
