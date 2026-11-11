package frc.demacia.path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.trapezoid.DemaciaTrapezoid;
import frc.demacia.path.utils.PathUtils;
import frc.demacia.utils.log.Log;

public class SegmentFollow {
    private DemaciaTrapezoid driveTrapezoid;
    private ProfiledPIDController rotationPid;

    public SegmentFollow() {
        driveTrapezoid = new DemaciaTrapezoid(PathConstants.MAX_VELOCITY, PathConstants.MAX_ACCELERATION);
        rotationPid = new ProfiledPIDController(1, 0, 0, new Constraints(PathConstants.MAX_ANGULAR_VELOCITY, PathConstants.MAX_ANGULAR_ACCELERATION));
    }

    public ChassisSpeeds getChassisSpeeds(SegmentBase currentSegment, Pose2d currentPose, ChassisSpeeds currentVelocity, double finalVel) {
        Translation2d currentVelVector = new Translation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
        Translation2d chassisPoseAsVector = currentPose.getTranslation();

        double angleError = MathUtil.angleModulus(currentSegment.getEndRotation().getRadians() - currentPose.getRotation().getRadians());
        double omega = rotationPid.calculate(angleError, currentVelocity.omegaRadiansPerSecond);
        // Log.log("angleError: " + angleError + " omega: " + omega);

        Translation2d calculatedVel;

        if(PathUtils.isLineSegment(currentSegment)){
            LineSegment lineSegment = (LineSegment) currentSegment;
            calculatedVel = calcVelInLineSegment(lineSegment, chassisPoseAsVector, currentVelVector, finalVel);
        }
        
        else{
            ArcSegment ArcSegment = (ArcSegment) currentSegment;
            calculatedVel = calcVelInArcSegment(ArcSegment, chassisPoseAsVector, currentVelVector, finalVel);
        }
        // LogManager.log("omega: " + omega);
        return new ChassisSpeeds(calculatedVel.getX(), calculatedVel.getY(), omega);
    }

    private Translation2d calcVelInLineSegment(LineSegment lineSegment, Translation2d chassisPoseAsVector, Translation2d currentVelVector, double finalVel) {
        Translation2d vectorToFinish = lineSegment.getEndPose().minus(chassisPoseAsVector);
        double vel = driveTrapezoid.nextVelocity(vectorToFinish.getNorm(), currentVelVector.getNorm(), finalVel);
        Rotation2d fixedHading = new Rotation2d(2 * vectorToFinish.getAngle().getRadians() - lineSegment.getTranslation().getAngle().getRadians());
        // Log.log("lineCalculatedVel: Norm: " + vel + " Rotation2d: " + fixeHading + " " + new Translation2d(vel, fixeHading));
        return new Translation2d(vel, fixedHading);
    }

    private Translation2d calcVelInArcSegment(ArcSegment arcSegment, Translation2d chassisPoseAsVector, Translation2d currentVelVector, double finalVel){
        
        Translation2d centerToChassis = chassisPoseAsVector.minus(arcSegment.getCenter());
        double baseHeading = centerToChassis.getAngle().getRadians() + (arcSegment.getIsLeft() ? Math.PI/2 : -Math.PI/2);
        double headingVelOmega = currentVelVector.getNorm() / PathConstants.radius;
        double heading = baseHeading + (arcSegment.getIsLeft() ? headingVelOmega * 0.02 : - headingVelOmega * 0.02);
        double distanceLeft = Math.abs(arcSegment.getFinishAngle().getRadians() - baseHeading) * PathConstants.radius;
        double velocity = driveTrapezoid.nextVelocity(distanceLeft, currentVelVector.getNorm(), finalVel);
        // Log.log("arcCalculatedVel: Norm: " + velocity + " Rotation2d: " + heading + " " + new Translation2d(velocity, new Rotation2d(heading)));
        return new Translation2d(velocity, new Rotation2d(heading));
    }
}
