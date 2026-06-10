package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;

public class SegmantBase {

    private Pose2d startPose;
    private Pose2d endPose;

    public SegmantBase(Pose2d startPose, Pose2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public Pose2d getEndPose() {
        return endPose;
    }
}
