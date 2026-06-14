package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.log.LogManager;

public class DemaciaTrajectoryGood {

    private List<Pose2d> demaciaPathPoint;
    private List<Pose2d> pathPoint;
    private List<LineSegment> lineSegmants;
    private List<ArcSegment> arcSegmants;
    private List<SegmantBase> segments;

    private SegmantFollow segmentFollow;
    private int currentSegmentIndex;
    private SegmantBase currentSegment;
    private boolean isFinishedTrajectory;

    public DemaciaTrajectoryGood(List<Pose2d> demaciaPoints) {
        this.demaciaPathPoint = demaciaPoints;
        this.pathPoint = new ArrayList<Pose2d>();
        this.lineSegmants = new ArrayList<LineSegment>();
        this.arcSegmants = new ArrayList<ArcSegment>();
        this.segments = new ArrayList<SegmantBase>();

        this.segmentFollow = new SegmantFollow();
        this.isFinishedTrajectory = false;

        if (demaciaPathPoint.size() < 2) {
            LogManager.log("Not enough points to build a trajectory");
            this.isFinishedTrajectory = true;
            return;
        }

        buildPath();

        this.currentSegmentIndex = 0;
        this.currentSegment = segments.get(this.currentSegmentIndex);
        LogManager.log("Segments size: " + segments.size());
        for (int i = 0; i < segments.size(); i++){
            LogManager.log(segments.get(i));
        }
        LogManager.log("Line Segments size: " + lineSegmants.size());
        for (int i = 0; i < lineSegmants.size(); i++){
            LogManager.log(lineSegmants.get(i));
        }
        LogManager.log("Arc Segments size: " + arcSegmants.size());
        for (int i = 0; i < arcSegmants.size(); i++){
            LogManager.log(arcSegmants.get(i));
        }
        LogManager.log("Path Point size: " + pathPoint.size());
        for (int i = 0; i < pathPoint.size(); i++){
            LogManager.log(pathPoint.get(i));
        }
    }

    private void buildPath() {

        
        for (int i = 1; i < demaciaPathPoint.size() - 1; i++) {
            Translation2d from = demaciaPathPoint.get(i - 1).getTranslation();
            Translation2d to = demaciaPathPoint.get((i+1)).getTranslation();
            Translation2d center = demaciaPathPoint.get(i).getTranslation();

            ArcSegment arcSegment = new ArcSegment(from, to, center);
            arcSegmants.add(arcSegment);    
        }

        for(int i = 0; i < demaciaPathPoint.size() / 2; i++){
            segments.add(new LineSegment(demaciaPathPoint.get(i).getTranslation(), demaciaPathPoint.get(i+1).getTranslation()));
            segments.add(new ArcSegment(demaciaPathPoint.get(i+1).getTranslation(), demaciaPathPoint.get(i+2).getTranslation(), arcSegmants.get(i).getCenterCircle()));
        }
        segments.add(new LineSegment(demaciaPathPoint.get(demaciaPathPoint.size() - 2).getTranslation(), demaciaPathPoint.get(demaciaPathPoint.size() - 1).getTranslation()));

        for (int i = 0; i < demaciaPathPoint.size() - 1; i++) {
            for (int j = 0; j < demaciaPathPoint.size(); j++) {
                lineSegmants.add(new LineSegment(demaciaPathPoint.get(i).getTranslation(), demaciaPathPoint.get(i + 1).getTranslation()));
            }
        }

        for (int i = 1; i < arcSegmants.size(); i++) {
            Translation2d from = demaciaPathPoint.get(i - 1).getTranslation();
            pathPoint.addAll(LegCalculator.returnPoint(from, arcSegmants.get(i), 0.5));
        }  

        if(!segments.isEmpty()){
            currentSegmentIndex = 0;
            currentSegment = segments.get(0);
        }else{
            isFinishedTrajectory = true;
            LogManager.log("failed to build path");
        }
    }

    public ChassisSpeeds calculateSpeeds(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        
        double finishVelocity = currentSegmentIndex == segments.size() - 1 ? 0 : PathConstants.MAX_VELOCITY;
        ChassisSpeeds speeds = segmentFollow.getChassisSpeeds(segments.get(currentSegmentIndex), currentPose, currentSpeeds, finishVelocity);
        
        if(isFinishedSegment(currentSpeeds, currentPose, currentSegment)){
            if(currentSegmentIndex == segments.size() - 1) {
                isFinishedTrajectory = true;
                return new ChassisSpeeds(0, 0, 0);
            }
            currentSegmentIndex++;
            currentSegment = segments.get(currentSegmentIndex);
        }
        
        return speeds;
        // if(isFinishedSegment(currentSpeeds, currentPose, currentSegment)){
        //     if(currentSegmentIndex == segments.size() - 1) {
        //         isFinishedTrajectory = true;
        //         return new ChassisSpeeds(0, 0, 0);
        //     }
        //     currentSegmentIndex++;
        //     currentSegment = segments.get(currentSegmentIndex);
        // }

        // return speeds;
    }

    private boolean isFinishedSegment(ChassisSpeeds currentSpeeds, Pose2d currentPose, SegmantBase segment) {

        double distanceFromFinishPoint = currentSegment.getEndPose().getDistance(currentPose.getTranslation());
        Rotation2d currentVelocityHeading = Rotation2d.kZero;
        if (!(currentSpeeds.vxMetersPerSecond == 0 && currentSpeeds.vyMetersPerSecond == 0))
            currentVelocityHeading = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond).getAngle();

        if(currentSegment instanceof LineSegment){

            LineSegment lineSegment = (LineSegment) currentSegment;

            boolean isVelocityHeadingTowardesFinishPoint = PathUtils.isVelocityHeadingInRange(currentVelocityHeading, lineSegment.getStartToEndVector().getAngle());
            if(currentSegmentIndex == segments.size() -1){
                return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_FINAL_POINT);
            }
            // LogManager.log((distanceFromFinishPoint < PathsConstants.MAX_POSITION_THRESHOLD_DURING_PATH) + " " + (distanceFromFinishPoint < (PathsConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) + " " +  isVelocityHeadingTowardesFinishPoint);
            return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH) || ((distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) && isVelocityHeadingTowardesFinishPoint);
            
            
        }

        else{
            ArcSegment arcSegment = (ArcSegment) currentSegment;
            Translation2d centerToFinish = arcSegment.getCenterCircle().minus(arcSegment.getEndPose());
            Rotation2d wantedVelocityHeading = centerToFinish.getAngle().minus(Rotation2d.kCW_90deg);
            boolean isHeadingTowardesNextSegment = PathUtils.isVelocityHeadingInRange(currentVelocityHeading, wantedVelocityHeading);
            // LogManager.log("isFinishedSegment " + (distanceFromFinishPoint < pathConstans.MAX_POSITION_THRESHOLD_DURING_PATH) + " " + (distanceFromFinishPoint < (pathConstans.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) + " "  + "isHeadingTowardesNextSegment " + isHeadingTowardesNextSegment + " " + currentVelocityHeading + "currentVelocityHeading" + " " + "wantedVelocityHeading" + wantedVelocityHeading + " " + "distanceFromFinishPoint" + distanceFromFinishPoint);
            return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH) || ((distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) && isHeadingTowardesNextSegment);
        }
    }

    public Pose2d getEndPoint(){
        return demaciaPathPoint.get(demaciaPathPoint.size() -1);
    }
}
