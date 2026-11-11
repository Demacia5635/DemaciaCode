package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LineSegment extends SegmentBase {
    private Translation2d startToEndVector;

    public LineSegment(Translation2d startPoint, Pose2d endPoint, double endVelocity, double maxVelocity) {
        super(startPoint, endPoint, endVelocity, maxVelocity);
        this.startToEndVector = endPoint.getTranslation().minus(startPoint);
    }

    @Override
    public Translation2d getTranslation() {
        return startToEndVector;
    }
}
