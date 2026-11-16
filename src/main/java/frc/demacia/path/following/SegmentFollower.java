package frc.demacia.path.following;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.PathSegment;
import frc.demacia.utils.log.Log;

/**
 * Everything that depends on the segment type (line or arc):
 * the speeds needed to follow it, and when it is finished.
 */
public class SegmentFollower {

    private final DemaciaTrapezoid velocityProfile;
    private final ProfiledPIDController headingController;

    public SegmentFollower() {
        velocityProfile = new DemaciaTrapezoid(
                PathConstants.MAX_VELOCITY_MPS, PathConstants.MAX_ACCELERATION_MPS2);
        headingController = new ProfiledPIDController(1, 0, 0, new Constraints(
                PathConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC, PathConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2));
    }

    // ---------------------------------------------------------------- speeds

    /**
     * @param segment          the segment currently being followed
     * @param currentPose      current robot pose on the field
     * @param currentSpeeds    current field-relative chassis speeds
     * @param endVelocityMps   wanted speed at the end of the segment
     */
    public ChassisSpeeds calculateChassisSpeeds(PathSegment segment, Pose2d currentPose,
            ChassisSpeeds currentSpeeds, double endVelocityMps) {
        Translation2d currentVelocityVector = new Translation2d(
                currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        Translation2d currentPosition = currentPose.getTranslation();

        double headingErrorRad = MathUtil.angleModulus(
                segment.getEndPose().getRotation().getRadians() - currentPose.getRotation().getRadians());
        double angularVelocityCommand = headingController.calculate(headingErrorRad, currentSpeeds.omegaRadiansPerSecond);

        Translation2d velocityCommand;
        if (segment instanceof LineSegment lineSegment) {
            velocityCommand = calculateVelocityOnLine(lineSegment, currentPosition, currentVelocityVector, endVelocityMps);
        } else {
            velocityCommand = calculateVelocityOnArc((ArcSegment) segment, currentPosition, currentVelocityVector, endVelocityMps);
        }

        return new ChassisSpeeds(velocityCommand.getX(), velocityCommand.getY(), angularVelocityCommand);
    }

    private Translation2d calculateVelocityOnLine(LineSegment line, Translation2d currentPosition,
            Translation2d currentVelocityVector, double endVelocityMps) {
        Translation2d vectorToEnd = line.getEndPose().getTranslation().minus(currentPosition);
        double speed = velocityProfile.nextVelocity(
                vectorToEnd.getNorm(), currentVelocityVector.getNorm(), endVelocityMps);

        // Mirror the direction-to-end across the line direction, to steer back towards the line
        Rotation2d correctedHeading = new Rotation2d(
                2 * vectorToEnd.getAngle().getRadians() - line.getStartToEndVector().getAngle().getRadians());
        return new Translation2d(speed, correctedHeading);
    }

    private Translation2d calculateVelocityOnArc(ArcSegment arc, Translation2d currentPosition,
            Translation2d currentVelocityVector, double endVelocityMps) {
        Translation2d centerToRobot = currentPosition.minus(arc.getCenter());

        // The wanted direction of travel is tangent to the circle
        double tangentHeadingRad = centerToRobot.getAngle().getRadians()
                + (arc.isLeftTurn() ? Math.PI / 2 : -Math.PI / 2);

        // Turn the heading ahead by one control cycle of rotation around the center
        double angularSpeedAroundCenter = currentVelocityVector.getNorm() / arc.getRadiusMeters();
        double lookAheadHeadingRad = tangentHeadingRad + (arc.isLeftTurn()
                ? angularSpeedAroundCenter * PathConstants.DT
                : -angularSpeedAroundCenter * PathConstants.DT);

        double remainingArcLength = Math.abs(arc.getEndAngleFromCenter().getRadians() - tangentHeadingRad)
                * arc.getRadiusMeters();
        double speed = velocityProfile.nextVelocity(
                remainingArcLength, currentVelocityVector.getNorm(), endVelocityMps);

        return new Translation2d(speed, new Rotation2d(lookAheadHeadingRad));
    }

    // ---------------------------------------------------------------- finished check

    /**
     * @param segment       the segment currently being followed
     * @param currentPose   current robot pose on the field
     * @param currentSpeeds current field-relative chassis speeds
     * @param isLastSegment true if this is the last segment of the path
     */
    public boolean isFinished(PathSegment segment, Pose2d currentPose, ChassisSpeeds currentSpeeds,
            boolean isLastSegment) {
        Translation2d robotToSegmentEnd = segment.getEndPose().getTranslation().minus(currentPose.getTranslation());

        if (segment instanceof LineSegment lineSegment) {
            boolean isFinished = isLineFinished(lineSegment, robotToSegmentEnd, isLastSegment);
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
        return isArcFinished((ArcSegment) segment, currentVelocityHeading);
    }

    private boolean isLineFinished(LineSegment line, Translation2d robotToSegmentEnd, boolean isLastSegment) {
        double distanceToEnd = robotToSegmentEnd.getNorm();

        if (isLastSegment) {
            return distanceToEnd < PathConstants.FINAL_POINT_POSITION_THRESHOLD_METERS;
        }

        Rotation2d angleToEndRelativeToLine = robotToSegmentEnd.getAngle().minus(line.getStartToEndVector().getAngle());
        boolean isEndPointPassed = Math.abs(MathUtil.angleModulus(angleToEndRelativeToLine.getRadians()))
                > PathConstants.MAX_ANGLE_TO_CONSIDER_POINT_PASSED_RAD;

        return distanceToEnd < PathConstants.MID_PATH_POSITION_THRESHOLD_METERS || isEndPointPassed;
    }

    /** An arc is finished when the robot's velocity heading is close to the arc's exit direction. */
    private boolean isArcFinished(ArcSegment arc, Rotation2d currentVelocityHeading) {
        Translation2d centerToArcEnd = arc.getEndPose().getTranslation().minus(arc.getCenter());
        Rotation2d exitHeading = centerToArcEnd.getAngle()
                .plus(arc.isLeftTurn() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
        return Math.abs(currentVelocityHeading.minus(exitHeading).getRadians())
                < PathConstants.MAX_HEADING_ERROR_TO_FINISH_ARC_RAD;
    }
}
