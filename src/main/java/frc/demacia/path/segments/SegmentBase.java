package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SegmentBase {

    private Translation2d startPose;
    private Pose2d endPose;
    private double endVelocity;
    private double maxVelocity;

    public SegmentBase(Translation2d startPose, Pose2d endPose, double endVelocity, double maxVelocity) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.endVelocity = endVelocity;
        this.maxVelocity = maxVelocity;
    }

    public Translation2d getStartTranslation() {
        return startPose;
    }

    public Pose2d getEndPose() {
        return endPose;
    }
    

    public double getMaxVelocity(){return this.maxVelocity;}

    public double getEndVelocity(){return this.endVelocity;}

    @Override
    public String toString(){
        return "Start point: " + startPose + " Finish Point: " + endPose;
    }
}
