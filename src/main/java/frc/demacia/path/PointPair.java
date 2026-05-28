package frc.demacia.path;

public class PointPair<Pose2d> {
    private final Pose2d point1;
    private final Pose2d point2;

    public PointPair(Pose2d point1, Pose2d point2) {
        this.point1 = point1;
        this.point2 = point2;
    }

    public Pose2d getPoint1() {
        return point1;
    }

    public Pose2d getPoint2() {
        return point2;
    }

}