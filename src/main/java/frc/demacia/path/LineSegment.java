package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LineSegment extends SegmantBase {
    private Translation2d startToEndVector;
    private double FinalVelocity;
    public boolean isRight;

    public LineSegment(double FinalVelocity, Pose2d startPoint, Translation2d endPoint,boolean isRight) {
        super(startPoint, endPoint);
        this.isRight = isRight;
        this.startToEndVector = endPoint.minus(startPoint);
        this.FinalVelocity = FinalVelocity;
    }

    public LineSegment(Translation2d startPoint, Translation2d endPoint, boolean isRight) {
        this(pathConstans.MAX_VELOCITY, startPoint, endPoint,isRight);
    }

    public Translation2d getStartToEndVector() {
        return startToEndVector;
    }

    public double getFinalVelocity() {
        return FinalVelocity;
    }
}
