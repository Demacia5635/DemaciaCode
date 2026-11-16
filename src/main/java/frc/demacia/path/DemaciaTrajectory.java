package frc.demacia.path;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.builder.TrajectoryBuilder;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.following.SegmentFollower;
import frc.demacia.path.segments.PathSegment;
import frc.demacia.utils.log.Log;

/**
 * A path through a list of waypoints. Straight lines connect the waypoints and
 * every corner is rounded with a circular arc.
 * The path is built once by {@link TrajectoryBuilder}; this class follows it segment by segment.
 */
public class DemaciaTrajectory {

    /** The sequence the robot follows: line, arc, line, arc, ..., line. */
    private final List<PathSegment> pathSegments;
    /** First waypoint, where the robot is when the trajectory starts. */
    private final Pose2d startPose;
    private final SegmentFollower segmentFollower = new SegmentFollower();

    private int currentSegmentIndex = 0;
    private boolean isTrajectoryFinished;

    public DemaciaTrajectory(List<Pose2d> waypoints) {
        pathSegments = TrajectoryBuilder.build(waypoints, PathConstants.CORNER_ARC_RADIUS);
        startPose = waypoints.isEmpty() ? Pose2d.kZero : waypoints.get(0);
        isTrajectoryFinished = pathSegments.isEmpty();
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    /** Resets the trajectory so it is followed from the first segment. */
    public void start() {
        currentSegmentIndex = 0;
        isTrajectoryFinished = pathSegments.isEmpty();
    }

    public boolean isFinished() {
        return isTrajectoryFinished;
    }

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

        PathSegment currentSegment = pathSegments.get(currentSegmentIndex);
        boolean isLastSegment = currentSegmentIndex == pathSegments.size() - 1;
        double endVelocityMps = isLastSegment ? 0 : PathConstants.MAX_VELOCITY_MPS;

        ChassisSpeeds speeds = segmentFollower.calculateChassisSpeeds(
                currentSegment, currentPose, currentSpeeds, endVelocityMps);

        if (!segmentFollower.isFinished(currentSegment, currentPose, currentSpeeds, isLastSegment)) {
            return speeds;
        }

        if (isLastSegment) {
            double headingErrorRad = currentPose.getRotation()
                    .minus(currentSegment.getEndPose().getRotation()).getRadians();
            if (Math.abs(headingErrorRad) < PathConstants.FINAL_HEADING_TOLERANCE_RAD) {
                isTrajectoryFinished = true;
                return new ChassisSpeeds(0, 0, 0);
            }
            return new ChassisSpeeds(0, 0, speeds.omegaRadiansPerSecond);
        }

        Log.log("currentPose: " + currentPose + " speeds: " + speeds + " finished segment: " + currentSegment);

        currentSegmentIndex++;
        return speeds;
    }
}
