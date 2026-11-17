package frc.demacia.path;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.builder.TrajectoryBuilder;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.following.SegmentFollower;
import frc.demacia.path.segments.PathSegment;
import frc.demacia.utils.log.Log;

/**
 * A path through a list of waypoints, followed segment by segment. The segments are built by
 * {@link PathBuilder}; this class keeps track of which one the robot is on.
 */
public class DemaciaTrajectory {

    private final List<PathSegment> pathSegments;
    private final SegmentFollower segmentFollower = new SegmentFollower();

    private int currentSegmentIndex = 0;
    private boolean isTrajectoryFinished;

    public DemaciaTrajectory(List<Pose2d> waypoints) {
        pathSegments = TrajectoryBuilder.build(waypoints,PathConstants.CORNER_ARC_RADIUS);
        isTrajectoryFinished = pathSegments.isEmpty();

    //     for (int i = 0; i < pathSegments.size(); i++) {
    //         Log.log("pathSegments.get(" + i + "): " + pathSegments.get(i));
    //     }
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

        Translation2d currentVelocity = new Translation2d(
                currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        if (!currentSegment.isFinished()) { //currentPose, currentVelocity, isLastSegment
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

        

        currentSegmentIndex++;
        // Log.log("currentPose: " + currentPose + " speeds: " + speeds + " finished segment: " + currentSegment);
        return speeds;
    }
}
