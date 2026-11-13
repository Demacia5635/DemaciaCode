package frc.demacia.path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.following.TrapezoidVelocityProfile;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.PathSegment;

/** Calculates the field-relative chassis speeds needed to follow one segment. */
public class SegmentFollower {

    private final TrapezoidVelocityProfile velocityProfile;
    private final ProfiledPIDController headingController;

    public SegmentFollower() {
        velocityProfile = new TrapezoidVelocityProfile(
                PathConstants.MAX_VELOCITY_MPS, PathConstants.MAX_ACCELERATION_MPS2);
        headingController = new ProfiledPIDController(1, 0, 0, new Constraints(
                PathConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC, PathConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2));
    }

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
        double speed = velocityProfile.calculateNextVelocity(
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
                ? angularSpeedAroundCenter * PathConstants.CONTROL_LOOP_PERIOD_SECONDS
                : -angularSpeedAroundCenter * PathConstants.CONTROL_LOOP_PERIOD_SECONDS);

        double remainingArcLength = Math.abs(arc.getEndAngleFromCenter().getRadians() - tangentHeadingRad)
                * arc.getRadiusMeters();
        double speed = velocityProfile.calculateNextVelocity(
                remainingArcLength, currentVelocityVector.getNorm(), endVelocityMps);

        return new Translation2d(speed, new Rotation2d(lookAheadHeadingRad));
    }
}
