package frc.demacia.path;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.segments.BaseSegment;
import frc.demacia.utils.log.Log;

/**
 * Drives a DemaciaTrajectory, one 20 ms cycle at a time.
 * TrajectoryBuilder plans the route once. This class runs it: it asks the current
 * line or arc for a velocity, adds a turning speed so the robot reaches the segment's end heading
 * when it reaches the segment's end point, and moves on
 * to the next segment when the current one is finished.
 * FollowTrajectoryCommand calls it every cycle.
 *
 * <p>Use it like this: call {@link #start} once, then call {@link #calculate} every cycle
 * until {@link #isFinished} is true.
 */
public class PathFollower {

    // The values we show in the log. They are static because Log.putData adds a new
    // log entry every time it is called, so we register them only once for the whole program.
    private static double loggedSegmentIndex;
    private static double loggedVx; // m/s, requested field speed
    private static double loggedVy; // m/s
    private static double loggedOmega; // rad/s

    static {
        Log.putData("Path/segmentIndex", () -> loggedSegmentIndex);
        Log.putData("Path/requestedVx", () -> loggedVx);
        Log.putData("Path/requestedVy", () -> loggedVy);
        Log.putData("Path/omega", () -> loggedOmega);
    }

    private List<BaseSegment> segments = List.of();
    private double maxAngularSpeed; // rad/s
    private double finalHeadingTolerance; // rad

    private int currentSegmentIndex;
    private boolean isFinished = true;

    /**
     * Starts a new run from the first segment. Can be called again to restart.
     * An empty trajectory is finished right away.
     *
     * @param trajectory the route to follow
     */
    public void start(DemaciaTrajectory trajectory) {
        segments = trajectory.getSegments();
        finalHeadingTolerance = trajectory.getFinalHeadingTolerance();
        maxAngularSpeed = trajectory.getMaxAngularSpeed();
        isFinished = segments.isEmpty();
        currentSegmentIndex = 0;
    }

    /**
     * @return true when the last segment is done and the final heading is reached
     *         (also true before {@link #start} is called)
     */
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * Works out the speeds to drive for this cycle. Call it every 20 ms.
     * If the pose or the speeds contain NaN or infinity, it returns zero speeds
     * and changes nothing else, so the robot just waits for good data.
     * After the path is finished it returns zero speeds.
     *
     * @param currentPose robot pose on the field (meters, rotation)
     * @param fieldSpeeds measured robot speeds, field-relative (m/s, m/s, rad/s)
     * @return the wanted speeds, field-relative (m/s, m/s, rad/s)
     */
    public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds fieldSpeeds) {
        if (isFinished) {
            return logAndReturn(new ChassisSpeeds());
        }
        if (!PathUtils.allNumbersAreFinite(currentPose, fieldSpeeds)) {
            Log.log("path: pose or speeds not finite, sending zero this cycle");
            return logAndReturn(new ChassisSpeeds());
        }

        BaseSegment segment = segments.get(currentSegmentIndex);
        Translation2d position = currentPose.getTranslation();
        double currentVelocity = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        Translation2d velocity = segment.getVelocity(position,
                new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond));

        double omega = calculateOmega(currentPose, segment, position, velocity.getNorm());
        ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
        loggedSegmentIndex = currentSegmentIndex;

        if (!segment.isFinished(position)) {
            return logAndReturn(speeds);
        }
        // A stop point (end speed 0) is done only once the robot really stopped (below MAX_VELOCITY_TO_CONSIDER_STOPPED).
        // Until then keep braking and turning, but do not move on.
        if (segment.getEndVelocity() == 0 && currentVelocity > PathConstants.MAX_VELOCITY_TO_CONSIDER_STOPPED) {
            return logAndReturn(new ChassisSpeeds(0, 0, omega));
        }
        // Move to the next segment. We return THIS cycle's speeds of the segment that just ended
        // ("speeds" was computed before the index changed), so there is no zero-speed gap between segments.
        if (currentSegmentIndex < segments.size() - 1) {
            currentSegmentIndex++;
            Log.log("path: segment reached its end, next segment " + currentSegmentIndex);
            return logAndReturn(speeds);
        }
        // Last segment: translation is done, but the path only ends when the heading is close enough.
        // Until then send no translation, only turning.
        double headingError = currentPose.getRotation().minus(segment.getEndPose().getRotation()).getRadians();
        if (Math.abs(headingError) < finalHeadingTolerance) {
            isFinished = true;
            Log.log("path: final heading reached, next segment none (finished)");
            return logAndReturn(new ChassisSpeeds());
        }
        return logAndReturn(new ChassisSpeeds(0, 0, omega));
    }

    // Turning speed for this cycle (rad/s).
    // Spread the heading change evenly over the distance left to the segment's end point,
    // so the robot reaches the end heading exactly when it reaches the end point:
    //   time left = distance left / speed  ->  omega = heading error / time left
    // When the robot is (almost) stopped or already at the point, there is no distance left to spread
    // the turn over, so it finishes the turn as fast as allowed (the whole error in one cycle, capped at max).
    private double calculateOmega(Pose2d currentPose, BaseSegment segment, Translation2d position, double speed) {
        double headingError = segment.getEndPose().getRotation().minus(currentPose.getRotation()).getRadians();
        double distanceLeft = segment.getDistanceLeft(position);
        double omega;
        if (speed > PathConstants.MAX_VELOCITY_TO_CONSIDER_STOPPED
                && distanceLeft > PathConstants.MIN_WAYPOINT_DISTANCE) {
            omega = headingError * speed / distanceLeft;
        } else {
            omega = headingError / PathConstants.DT;
        }
        return MathUtil.clamp(omega, -maxAngularSpeed, maxAngularSpeed);
    }

    // logs the speeds we are about to send, then returns them
    private static ChassisSpeeds logAndReturn(ChassisSpeeds speeds) {
        loggedVx = speeds.vxMetersPerSecond;
        loggedVy = speeds.vyMetersPerSecond;
        loggedOmega = speeds.omegaRadiansPerSecond;
        return speeds;
    }
}
