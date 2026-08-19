package frc.demacia.path.segments;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.demacia.path.constans.PathConstants;
import frc.demacia.path.trapzoid.DemaciaTrapezoid;
import frc.demacia.path.utils.PathUtils;
import frc.demacia.utils.log.LogManager;

public class SegmantFollow {
    private DemaciaTrapezoid driveTrapzoid;
    private ProfiledPIDController rotationPid;

    public SegmantFollow() {
        driveTrapzoid = new DemaciaTrapezoid(PathConstants.MAX_VELOCITY, PathConstants.MAX_ACCELERATION);
        rotationPid = new ProfiledPIDController(1, 0, 0, new Constraints(PathConstants.MAX_ANGULAR_VELOCITY, PathConstants.MAX_ANGULAR_ACCELERATION));
    }

    public ChassisSpeeds getChassisSpeeds(SegmantBase CurrentSegmant, Pose2d currentPose, ChassisSpeeds currentVelocity, double finalVel) {
        Translation2d currentVelVector = new Translation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
        Translation2d chassisPoseAsVector = currentPose.getTranslation();

        // double angleError = MathUtil.angleModulus(CurrentSegmant.getEndPose().getAngle().getRadians() - currentPose.getRotation().getRadians());
        // double omega = rotationPid.calculate(angleError, currentVelocity.omegaRadiansPerSecond);

        Translation2d calculatedVel;

        if(PathUtils.isLineSegment(CurrentSegmant)){
            LineSegment segmant = (LineSegment) CurrentSegmant;
            Translation2d VectorToFinish = segmant.getEndPose().minus(chassisPoseAsVector);
            double vel = driveTrapzoid.nextVelocity(VectorToFinish.getNorm(), currentVelVector.getNorm(), finalVel);
            Rotation2d fixeHading = new Rotation2d(2 * VectorToFinish.getAngle().getRadians() - segmant.getTranslation().getAngle().getRadians());
            LogManager.log("lineCalculatedVel: Norm: " + vel + " Rotation2d: " + fixeHading + " " + new Translation2d(vel, fixeHading));
            calculatedVel = new Translation2d(vel, fixeHading);
        }
        
        else{
            ArcSegment segmant = (ArcSegment) CurrentSegmant;
            Translation2d centerToChassis = chassisPoseAsVector.minus(segmant.getCenter());
            double baseHeading = centerToChassis.getAngle().getRadians() + (segmant.getIsLeft() ? Math.PI/2 : -Math.PI/2);
            double headingVelOmega = currentVelVector.getNorm() / PathConstants.radius;
            double heading = baseHeading + (segmant.getIsLeft() ? headingVelOmega * 0.02 : - headingVelOmega * 0.02);
            double distanceLeft = Math.abs(segmant.getFinishAngle().getRadians() - baseHeading) * PathConstants.radius;
            double velocity = driveTrapzoid.nextVelocity(distanceLeft, currentVelVector.getNorm(), finalVel);
            LogManager.log("arcCalculatedVel: Norm: " + velocity + " Rotation2d: " + heading + " " + new Translation2d(velocity, new Rotation2d(heading)));
            calculatedVel = new Translation2d(velocity, new Rotation2d(heading));
        }
        // LogManager.log("omega: " + omega);
        return new ChassisSpeeds(calculatedVel.getX(), calculatedVel.getY(), 0);
    }

}
