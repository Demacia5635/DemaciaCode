package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SegmentBase {

    private Translation2d startPose;
    private Translation2d endPose;
    private Rotation2d endRotation;

    public SegmentBase(Translation2d startPose, Translation2d endPose, Rotation2d endRotation) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.endRotation = endRotation;
    }

    public SegmentBase(Translation2d startPose, Pose2d endPose){
        this(startPose, endPose.getTranslation(), endPose.getRotation());
    }

    public Translation2d getStartPose() {
        return startPose;
    }

    public Translation2d getEndPose() {
        return endPose;
    }
    
    public Translation2d getTranslation(){
        return getStartPose().minus(getEndPose());
    }

    public Rotation2d getEndRotation(){
        return endRotation;
    }

    @Override
    public String toString(){
        return "Start point: " + startPose + " Finish Point: " + endPose;
    }
}
