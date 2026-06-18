package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Translation2d;

public class SegmantBase {

    private Translation2d startPose;
    private Translation2d endPose;

    public SegmantBase(Translation2d startPose, Translation2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
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

    @Override
    public String toString(){
        return "Start point: " + startPose + " Finish Point: " + endPose;
    }
}
