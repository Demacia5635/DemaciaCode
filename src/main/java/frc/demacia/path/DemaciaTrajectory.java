package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.calculator.LegCalculator;
import frc.demacia.path.calculator.ArcCalculator;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.SegmentBase;
import frc.demacia.path.segments.SegmentFollow;
import frc.demacia.path.utils.PathUtils;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.geometry.Transform2dDemacia;
import frc.demacia.utils.log.Log;

public class DemaciaTrajectory {

    private List<Pose2d> demaciaPathPoints;
    private List<Pose2d> pathPoints;
    private List<LineSegment> lineSegments;
    private List<ArcSegment> arcSegments;
    private List<SegmentBase> segments;
    private final double radius;

    private SegmentFollow segmentFollow;
    private int currentSegmentIndex;
    private SegmentBase currentSegment;
    private boolean isFinishedTrajectory;

    private int currentPathPointIndex;

    public DemaciaTrajectory(List<Pose2d> demaciaPoints) {
        this.demaciaPathPoints = demaciaPoints;
        this.pathPoints = new ArrayList<Pose2d>();
        this.lineSegments = new ArrayList<LineSegment>();
        this.arcSegments = new ArrayList<ArcSegment>();
        this.segments = new ArrayList<SegmentBase>();
        radius = PathConstants.radius;

        this.segmentFollow = new SegmentFollow();   
        this.isFinishedTrajectory = false;

        if (demaciaPathPoints.size() < 2) {
            Log.log("Not enough points to build a trajectory");
            this.isFinishedTrajectory = true;
            return;
        }
        else if (demaciaPathPoints.size() == 2) {
            LineSegment segment = new LineSegment(demaciaPathPoints.get(0).getTranslation(), demaciaPathPoints.get(1).getTranslation());
            segments.add(segment);
            pathPoints.add(demaciaPathPoints.get(0));
            pathPoints.add(demaciaPathPoints.get(1));
            Log.log("2 points, only line segment: " + segment);
        }
        else {
            Log.log("more than 2 points");
            buildPath();
        }
        this.currentSegmentIndex = 0;
        this.currentSegment = segments.get(this.currentSegmentIndex);
        this.currentPathPointIndex = 0;

        for (int i = 0; i < segments.size(); i++){
            Log.log("segments.get(" + i + "): " + segments.get(i));
        }
    }

    public void startTrajectory(){
        isFinishedTrajectory = false;
        this.currentSegmentIndex = 0;
        this.currentSegment = segments.get(this.currentSegmentIndex);
    }

    public boolean getIsFinishedTrajectory() {
        return isFinishedTrajectory;
    }

    private void buildPath() {
        buildLinesBetweenDemaciaPathPoints();
        buildArcs();
        buildPathPoints();
        buildSegments();
    }

    private void buildLinesBetweenDemaciaPathPoints(){
        for (int i = 0; i < demaciaPathPoints.size() - 1; i++) {
            LineSegment line = new LineSegment(demaciaPathPoints.get(i).getTranslation(), demaciaPathPoints.get(i + 1), , );
            this.lineSegments.add(line);
        }
    }

     private Pose2d getCurrentPoint(Pose2d currentPose) {

        Pose2d closestPoint = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose2d point : demaciaPathPoints) {

            if (point.getRotation() == null) {
                continue;
            }
            double distance = point.getTranslation().getDistance(currentPose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = point;
            }
        }

        return closestPoint;
    }

    private void buildArcs(){
        for (int i = 0; i < lineSegments.size() - 1; i++) {
            ArcSegment arc = ArcCalculator.calculateCenter(lineSegments.get(i).getStartPose(), lineSegments.get(i).getEndPose(), lineSegments.get(i+1).getEndPose(),radius);
            this.arcSegments.add(arc);
        }
    }

    private void buildPathPoints(){
        this.pathPoints.add(demaciaPathPoints.get(0));
        for (int i = 0; i < lineSegments.size() - 1; i++) {
            List<Pose2d> points = LegCalculator.returnPoint(lineSegments.get(i).getStartPose(), lineSegments.get(i).getEndPose(), arcSegments.get(i), radius);
            this.pathPoints.addAll(points);
        }
        this.pathPoints.add(demaciaPathPoints.get(demaciaPathPoints.size() - 1));
    }

    private void buildSegments(){
        for (int i = 0; i < pathPoints.size() - 2; i += 2) {
            LineSegment line = new LineSegment(pathPoints.get(i).getTranslation(), pathPoints.get(i + 1).getTranslation());
            segments.add(line);

            ArcSegment arc = new ArcSegment(pathPoints.get(i + 1).getTranslation(), pathPoints.get(i + 2).getTranslation(), arcSegments.get(i / 2).getCenter(), arcSegments.get(i / 2).getIsLeft(), radius);
            segments.add(arc);
        }
        segments.add(new LineSegment(pathPoints.get(pathPoints.size() - 2).getTranslation(), pathPoints.get(pathPoints.size() - 1).getTranslation()));
    }

    public ChassisSpeeds calculateSpeeds(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        
        double finishVelocity = currentSegmentIndex == segments.size() - 1 ? 0 : PathConstants.MAX_VELOCITY;
        ChassisSpeeds speeds = segmentFollow.getChassisSpeeds(segments.get(currentSegmentIndex), currentPose, currentSpeeds, finishVelocity);

        // Log.log("wanted speed: "+speeds+" \ncurrentSpeeds: "+currentSpeeds);
        if(currentSegment instanceof ArcSegment){
            ArcSegment arcSegment = (ArcSegment) currentSegment;
            Translation2d centerToChassis = currentPose.getTranslation().minus(arcSegment.getCenter());

        }        

        if(!isFinishedSegment(currentSpeeds, currentPose, currentSegment)) 
            return speeds;

        Log.log("currentPose: " + currentPose + " speeds: " + speeds + " finished segment: " + currentSegment);

        if(currentSegmentIndex == segments.size() - 1) {
            isFinishedTrajectory = true;
            return new ChassisSpeeds(0, 0, 0);
        }
        currentSegmentIndex++;
        // currentPathPointIndex++;
        currentSegment = segments.get(currentSegmentIndex);
        
        return speeds;
    }

    private boolean isFinishedSegment(ChassisSpeeds currentSpeeds, Pose2d currentPose, SegmentBase segment) {

        Translation2d toEnd = segment.getEndPose().minus(currentPose.getTranslation());

        Rotation2d currentVelocityHeading = Rotation2d.kZero;
        if (!(currentSpeeds.vxMetersPerSecond == 0 && currentSpeeds.vyMetersPerSecond == 0))
            currentVelocityHeading = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond).getAngle();

        if(PathUtils.isLineSegment(currentSegment)){

            LineSegment lineSegment = (LineSegment) currentSegment;
            if(isFinishedLineSegment(lineSegment, toEnd)) Log.log(toEnd.getNorm() + " Finished Line segmant distance");
            return isFinishedLineSegment(lineSegment, toEnd);

        }else{
            
            ArcSegment arcSegment = (ArcSegment) currentSegment;
            return isFinishedArcSegment(arcSegment, currentVelocityHeading);
        }
    }

    private boolean isFinishedLineSegment(LineSegment lineSegment, Translation2d toEnd) {

        double distanceFromFinishPoint = toEnd.getNorm();
        
        Rotation2d toEndRot = toEnd.getAngle().minus(lineSegment.getTranslation().getAngle());
        boolean passed = Math.abs(MathUtil.angleModulus(toEndRot.getRadians())) > PathConstants.MAX_ANGLE_TO_PASS;
        // boolean isVelocityHeadingTowardsFinishPoint = PathUtils.isVelocityHeadingInRange(currentVelocityHeading, lineSegment.getTranslation().getAngle());
        if(currentSegmentIndex == segments.size() -1){
            return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_FINAL_POINT);
        }
        return (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH) || passed;
    }

    private boolean isFinishedArcSegment(ArcSegment arcSegment, Rotation2d currentVelocityHeading){

        Translation2d centerToFinish = arcSegment.getEndPose().minus(arcSegment.getCenter());
        Rotation2d wantedVelocityHeading = centerToFinish.getAngle().plus(arcSegment.getIsLeft() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
        boolean isHeadingTowardesNextSegment = PathUtils.isVelocityHeadingInRange(currentVelocityHeading, wantedVelocityHeading);
        // Log.log("isFinishedSegment " + (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH) + " " + (distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3)) + " isHeadingTowardesNextSegment " + isHeadingTowardesNextSegment + " " + currentVelocityHeading + "currentVelocityHeading wantedVelocityHeading" + wantedVelocityHeading + " distanceFromFinishPoint" + distanceFromFinishPoint + " EndPoint: " + arcSegment.getEndPose() + " Current segment: " + currentSegment);
        // Log.log("isHeadingTowardesNextSegment: " + isHeadingTowardesNextSegment + " currentVelocityHeading: " + currentVelocityHeading + " wantedVelocityHeading: " + wantedVelocityHeading);
        return isHeadingTowardesNextSegment; // && (distanceFromFinishPoint < (PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH * 3))) || (distanceFromFinishPoint < PathConstants.MAX_POSITION_THRESHOLD_DURING_PATH);

    }
}
