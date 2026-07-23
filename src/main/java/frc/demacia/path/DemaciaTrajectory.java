package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.calclator.LegCalculator;
import frc.demacia.path.calclator.arcCalculator;
import frc.demacia.path.constans.PathConstants;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.SegmantBase;
import frc.demacia.path.segments.SegmantFollow;
import frc.demacia.path.utils.PathUtils;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;

public class DemaciaTrajectory {

    private List<Translation2d> demaciaPathPoints;
    private List<Translation2d> pathPoints;
    private List<LineSegment> lineSegments;
    private List<ArcSegment> arcSegmants;
    private List<SegmantBase> segments;
    private final double radius;

    private SegmantFollow segmentFollow;
    private int currentSegmentIndex;
    private SegmantBase currentSegment;
    public boolean isFinishedTrajectory;

    public DemaciaTrajectory(List<Translation2d> demaciaPoints) {
        this.demaciaPathPoints = demaciaPoints;
        this.pathPoints = new ArrayList<Translation2d>();
        this.lineSegments = new ArrayList<LineSegment>();
        this.arcSegmants = new ArrayList<ArcSegment>();
        this.segments = new ArrayList<SegmantBase>();
        radius = PathConstants.radius;

        this.segmentFollow = new SegmantFollow();
        this.isFinishedTrajectory = false;

        if (demaciaPathPoints.size() < 2) {
            LogManager.log("Not enough points to build a trajectory");
            this.isFinishedTrajectory = true;
            return;
        }
        else if (demaciaPathPoints.size() == 2) {
            LineSegment segment = new LineSegment(demaciaPathPoints.get(0), demaciaPathPoints.get(1));
            segments.add(segment);
        }
        else {
            buildPath();
        }
        this.currentSegmentIndex = 0;
        this.currentSegment = segments.get(this.currentSegmentIndex);

        for (int i = 0; i < segments.size(); i++){
            LogManager.log("segments.get(" + i + "): " + segments.get(i));
        }
        for (int i = 0; i < lineSegments.size(); i++){
            LogManager.log("lineSegments.get(" + i + "): " + lineSegments.get(i));
        }

    }

    private void buildPath() {
        for (int i = 0; i < demaciaPathPoints.size() - 1; i++) {
            LineSegment line = new LineSegment(demaciaPathPoints.get(i), demaciaPathPoints.get(i + 1));
            this.lineSegments.add(line);
        }

        for (int i = 0; i < lineSegments.size() - 1; i++) {
            ArcSegment arc = arcCalculator.calclateCenter(lineSegments.get(i).getStartPose(), lineSegments.get(i).getEndPose(), lineSegments.get(i+1).getEndPose(),radius);
            this.arcSegmants.add(arc);
        }

        this.pathPoints.add(demaciaPathPoints.get(0));
        for (int i = 0; i < lineSegments.size() - 1; i++) {
            List<Translation2d> points = LegCalculator.returnPoint(lineSegments.get(i).getStartPose(), lineSegments.get(i).getEndPose(), arcSegmants.get(i), radius);
            this.pathPoints.addAll(points);
        }
        this.pathPoints.add(demaciaPathPoints.get(demaciaPathPoints.size() - 1));

        for (int i = 0; i < pathPoints.size() - 2; i += 2) {
            LineSegment line = new LineSegment(pathPoints.get(i), pathPoints.get(i + 1));
            segments.add(line);

            ArcSegment arc = new ArcSegment(pathPoints.get(i + 1), pathPoints.get(i + 2), arcSegmants.get(i / 2).getCenter(), arcSegmants.get(i / 2).getIsLeft(), radius);
            segments.add(arc);
        }
        segments.add(new LineSegment(pathPoints.get(pathPoints.size() - 2), pathPoints.get(pathPoints.size() - 1)));
    }

    public ChassisSpeeds calculateSpeeds(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        
        double finishVelocity = currentSegmentIndex == segments.size() - 1 ? 0 : PathConstants.MAX_VELOCITY;
        // LogManager.log("currentSegmentIndex:" + currentSegmentIndex + " segments: " + segments.get(currentSegmentIndex) + " finishVelocity:" + finishVelocity);
        ChassisSpeeds speeds = segmentFollow.getChassisSpeeds(segments.get(currentSegmentIndex), currentPose, currentSpeeds, finishVelocity);
        // LogManager.log("speeds: " + speeds);// + " currentSegmentIndex: " + currentSegmentIndex + " segments.get(currentSegmentIndex): " + segments.get(currentSegmentIndex) + " currentPose: " + currentPose + " currentSpeeds: " + currentSpeeds + " finishVelocity: " + finishVelocity);
        if(isFinishedSegment(currentSpeeds, currentPose, currentSegment)){
            if(currentSegmentIndex == segments.size() - 1) {
                isFinishedTrajectory = true;
                return new ChassisSpeeds(0, 0, 0);
            }
            currentSegmentIndex++;
            currentSegment = segments.get(currentSegmentIndex);
        }
        // LogManager.log("wanted speed" + speeds + " current speed: " + chassis.getVelocityAsVector());
        return speeds;
    }

    private boolean isFinishedSegment(ChassisSpeeds currentSpeeds, Pose2d currentPose, SegmantBase segment) {

        double distanceFromFinishPoint = currentSegment.getEndPose().getDistance(currentPose.getTranslation());
        Rotation2d currentVelocityHeading = Rotation2d.kZero;
        if (!(currentSpeeds.vxMetersPerSecond == 0 && currentSpeeds.vyMetersPerSecond == 0))
            currentVelocityHeading = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond).getAngle();

        if(currentSegment instanceof LineSegment){

            LineSegment lineSegment = (LineSegment) currentSegment;

            boolean isVelocityHeadingTowardesFinishPoint = PathUtils.isVelocityHeadingInRange(currentVelocityHeading, lineSegment.getTranslation().getAngle());
            if(currentSegmentIndex == segments.size() -1){
                return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_FINAL_POINT);
            }
            return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH);// || ((distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) && isVelocityHeadingTowardesFinishPoint);
        }

        else{
            ArcSegment arcSegment = (ArcSegment) currentSegment;
            Translation2d centerToFinish = arcSegment.getCenter().minus(arcSegment.getEndPose());
            Rotation2d wantedVelocityHeading = centerToFinish.getAngle().minus(Rotation2d.kCW_90deg);
            boolean isHeadingTowardesNextSegment = PathUtils.isVelocityHeadingInRange(currentVelocityHeading, wantedVelocityHeading);
            LogManager.log("isFinishedSegment " + (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH) + " " + (distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) + " isHeadingTowardesNextSegment " + isHeadingTowardesNextSegment + " " + currentVelocityHeading + "currentVelocityHeading wantedVelocityHeading" + wantedVelocityHeading + " distanceFromFinishPoint" + distanceFromFinishPoint + " EndPoint: " + arcSegment.getEndPose() + " Current Segmant: " + currentSegment);
            return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH) || ((distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) && isHeadingTowardesNextSegment);
        }
    }

    public Translation2d getEndPoint(){
        return demaciaPathPoints.get(demaciaPathPoints.size() -1);
    }
}
