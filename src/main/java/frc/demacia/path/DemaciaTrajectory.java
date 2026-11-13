package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.geometry.CornerArcCalculator;
import frc.demacia.path.geometry.TangentCalculator;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.PathSegment;
import frc.demacia.path.segments.SegmentFollower;
import frc.demacia.path.utils.PathUtils;
import frc.demacia.utils.log.Log;

/**
 * A path through a list of waypoints. Straight lines connect the waypoints and
 * every corner is rounded with a circular arc.
 */
public class DemaciaTrajectory {

    private final List<Pose2d> waypoints;
    /** Raw straight lines from each waypoint to the next one (before rounding the corners). */
    private final List<LineSegment> waypointLines = new ArrayList<>();
    /** One arc per corner (every waypoint except the first and the last). */
    private final List<ArcSegment> cornerArcs = new ArrayList<>();
    /** First waypoint, then [arc entry, arc exit] for each corner, then last waypoint. */
    private final List<Pose2d> roundedPathPoints = new ArrayList<>();
    /** The final sequence the robot follows: line, arc, line, arc, ..., line. */
    private final List<PathSegment> pathSegments = new ArrayList<>();

    private final double cornerRadius = PathConstants.CORNER_ARC_RADIUS;
    private final SegmentFollower segmentFollower = new SegmentFollower();

    private int currentSegmentIndex = 0;
    private boolean isTrajectoryFinished = false;

    public DemaciaTrajectory(List<Pose2d> waypoints) {
        this.waypoints = waypoints;

        if (waypoints.size() < 2) {
            Log.log("Not enough points to build a trajectory");
            isTrajectoryFinished = true;
            return;
        }

        if (waypoints.size() == 2) {
            LineSegment line = new LineSegment(waypoints.get(0).getTranslation(), waypoints.get(1));
            pathSegments.add(line);
            Log.log("2 points, only line segment: " + line);
        } else {
            Log.log("more than 2 points");
            buildPathWithCornerArcs();
        }

        for (int i = 0; i < pathSegments.size(); i++) {
            Log.log("pathSegments.get(" + i + "): " + pathSegments.get(i));
        }
    }

    /** Resets the trajectory so it is followed from the first segment. */
    public void start() {
        currentSegmentIndex = 0;
        isTrajectoryFinished = pathSegments.isEmpty();
    }

    public boolean isFinished() {
        return isTrajectoryFinished;
    }

    // ---------------------------------------------------------------- building

    private void buildPathWithCornerArcs() {
        buildLinesBetweenWaypoints();
        buildCornerArcs();
        buildRoundedPathPoints();
        buildSegmentsFromRoundedPoints();
    }

    private void buildLinesBetweenWaypoints() {
        for (int i = 0; i < waypoints.size() - 1; i++) {
            waypointLines.add(new LineSegment(waypoints.get(i).getTranslation(), waypoints.get(i + 1)));
        }
    }

    private void buildCornerArcs() {
        for (int cornerIndex = 0; cornerIndex < waypointLines.size() - 1; cornerIndex++) {
            LineSegment lineIntoCorner = waypointLines.get(cornerIndex);
            LineSegment lineOutOfCorner = waypointLines.get(cornerIndex + 1);
            cornerArcs.add(CornerArcCalculator.calculateCornerArc(
                    lineIntoCorner.getStartTranslation(),
                    lineIntoCorner.getEndPose(),
                    lineOutOfCorner.getEndPose().getTranslation(),
                    cornerRadius));
        }
    }

    private void buildRoundedPathPoints() {
        roundedPathPoints.add(waypoints.get(0));
        for (int cornerIndex = 0; cornerIndex < cornerArcs.size(); cornerIndex++) {
            LineSegment lineIntoCorner = waypointLines.get(cornerIndex);
            roundedPathPoints.addAll(TangentCalculator.calculateArcEntryAndExitPoses(
                    lineIntoCorner.getStartTranslation(),
                    lineIntoCorner.getEndPose().getTranslation(),
                    cornerArcs.get(cornerIndex),
                    cornerRadius));
        }
        roundedPathPoints.add(waypoints.get(waypoints.size() - 1));
    }

    private void buildSegmentsFromRoundedPoints() {
        for (int cornerIndex = 0; cornerIndex < cornerArcs.size(); cornerIndex++) {
            int lineStartIndex = 2 * cornerIndex;
            Pose2d lineStart = roundedPathPoints.get(lineStartIndex);
            Pose2d arcEntry = roundedPathPoints.get(lineStartIndex + 1);
            Pose2d arcExit = roundedPathPoints.get(lineStartIndex + 2);
            ArcSegment cornerArc = cornerArcs.get(cornerIndex);

            pathSegments.add(new LineSegment(lineStart.getTranslation(), arcEntry));
            pathSegments.add(new ArcSegment(arcEntry.getTranslation(), arcExit,
                    cornerArc.getCenter(), cornerArc.isLeftTurn(), cornerRadius));
        }

        Pose2d lastLineStart = roundedPathPoints.get(roundedPathPoints.size() - 2);
        Pose2d lastPoint = roundedPathPoints.get(roundedPathPoints.size() - 1);
        pathSegments.add(new LineSegment(lastLineStart.getTranslation(), lastPoint));
    }

    // ---------------------------------------------------------------- following

    /**
     * Calculates the speeds to command now, and moves on to the next segment when the current one is finished.
     *
     * @param currentSpeeds current field-relative chassis speeds
     * @param currentPose   current robot pose on the field
     */
    public ChassisSpeeds calculateChassisSpeeds(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        if (isTrajectoryFinished) {
            return new ChassisSpeeds(0, 0, 0);
        }

        PathSegment currentSegment = getCurrentSegment();
        double endVelocityMps = isLastSegment() ? 0 : PathConstants.MAX_VELOCITY_MPS;
        ChassisSpeeds speeds = segmentFollower.calculateChassisSpeeds(
                currentSegment, currentPose, currentSpeeds, endVelocityMps);

        if (!isSegmentFinished(currentSpeeds, currentPose, currentSegment)) {
            return speeds;
        }

        if (isLastSegment()) {
            if(Math.abs(currentPose.getRotation().minus(pathSegments.get(currentSegmentIndex).getEndPose().getRotation()).getRadians()) < Math.toRadians(5)){ //TODO: put this in a constant
                isTrajectoryFinished = true;
                return new ChassisSpeeds(0, 0, 0);
            }
            return new ChassisSpeeds(0, 0, speeds.omegaRadiansPerSecond);
        }

        Log.log("currentPose: " + currentPose + " speeds: " + speeds + " finished segment: " + currentSegment);

        currentSegmentIndex++;
        return speeds;
    }

    private PathSegment getCurrentSegment() {
        return pathSegments.get(currentSegmentIndex);
    }

    private boolean isLastSegment() {
        return currentSegmentIndex == pathSegments.size() - 1;
    }

    private boolean isSegmentFinished(ChassisSpeeds currentSpeeds, Pose2d currentPose, PathSegment segment) {
        Translation2d robotToSegmentEnd = segment.getEndPose().getTranslation().minus(currentPose.getTranslation());

        if (segment instanceof LineSegment lineSegment) {
            boolean isFinished = isLineSegmentFinished(lineSegment, robotToSegmentEnd);
            if (isFinished) {
                Log.log(robotToSegmentEnd.getNorm() + " Finished line segment distance");
            }
            return isFinished;
        }

        Rotation2d currentVelocityHeading = Rotation2d.kZero;
        if (!(currentSpeeds.vxMetersPerSecond == 0 && currentSpeeds.vyMetersPerSecond == 0)) {
            currentVelocityHeading = new Translation2d(
                    currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond).getAngle();
        }
        return isArcSegmentFinished((ArcSegment) segment, currentVelocityHeading);
    }

    private boolean isLineSegmentFinished(LineSegment lineSegment, Translation2d robotToSegmentEnd) {
        double distanceToEnd = robotToSegmentEnd.getNorm();

        if (isLastSegment()) {
            return distanceToEnd < PathConstants.FINAL_POINT_POSITION_THRESHOLD_METERS;
        }

        Rotation2d angleToEndRelativeToLine = robotToSegmentEnd.getAngle().minus(lineSegment.getStartToEndVector().getAngle());
        boolean isEndPointPassed = Math.abs(MathUtil.angleModulus(angleToEndRelativeToLine.getRadians()))
                > PathConstants.MAX_ANGLE_TO_CONSIDER_POINT_PASSED_RAD;

        return distanceToEnd < PathConstants.MID_PATH_POSITION_THRESHOLD_METERS || isEndPointPassed;
    }

    private boolean isArcSegmentFinished(ArcSegment arcSegment, Rotation2d currentVelocityHeading) {
        Translation2d centerToArcEnd = arcSegment.getEndPose().getTranslation().minus(arcSegment.getCenter());
        Rotation2d exitHeading = centerToArcEnd.getAngle()
                .plus(arcSegment.isLeftTurn() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
        return PathUtils.isHeadingCloseToTarget(currentVelocityHeading, exitHeading);
    }
}
