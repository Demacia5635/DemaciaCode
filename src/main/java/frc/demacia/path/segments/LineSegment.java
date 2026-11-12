package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.constants.PathConstants;

public class LineSegment extends SegmentBase {
    private Translation2d startToEndVector;

    public LineSegment(Translation2d startPoint, Pose2d endPoint, double endVelocity, double maxVelocity) {
        super(startPoint, endPoint, endVelocity, maxVelocity);
        this.startToEndVector = endPoint.getTranslation().minus(startPoint);
    }

    public LineSegment(Translation2d startPoint, Pose2d endPoint){
        this(startPoint, endPoint, PathConstants.MAX_VELOCITY, PathConstants.MAX_VELOCITY);
    }


    public Translation2d getStartToEndVector() {
        return startToEndVector;
    }

}