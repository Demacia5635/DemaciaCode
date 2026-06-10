package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArcSegment extends SegmantBase {

    private Translation2d centerCircle;
    private Translation2d centerToStart;
    private Translation2d centerToFinish;

    public ArcSegment(Pose2d startingPoint, Pose2d finishPoint, Translation2d centerCircle ){
        super(startingPoint, finishPoint);
        this.centerCircle = centerCircle;
        this.centerToStart = startingPoint.getTranslation().minus(centerCircle);
        this.centerToFinish = finishPoint.getTranslation().minus(centerCircle);
    }

    public Translation2d getCenterCircle(){return this.centerCircle;}

    public Rotation2d getAngleBetweenRadius(){
        return centerToStart.getAngle().minus(centerToFinish.getAngle());
    }
}