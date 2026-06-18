package frc.demacia.path.segments;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.demacia.path.constans.PathConstants;
import frc.demacia.path.trapzoid.DemaciaTrapezoid;
import frc.demacia.path.utils.PathUtils;

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

        Translation2d calculatedVel;
        if(PathUtils.isLineSegment(CurrentSegmant)){
            LineSegment segmant = (LineSegment) CurrentSegmant;
            Translation2d VectorToFinish = segmant.getEndPose().minus(chassisPoseAsVector);
            double vel = driveTrapzoid.nextVelocity(VectorToFinish.getNorm(), currentVelVector.getNorm(), finalVel);
            Rotation2d fixeHading = VectorToFinish.getAngle();
            calculatedVel = new Translation2d(vel, fixeHading);
        }
        
        else{
            ArcSegment segmant = (ArcSegment) CurrentSegmant;
            Translation2d centerToChassis = chassisPoseAsVector.minus(segmant.getCenter());
            Rotation2d tanToCircleAngle = centerToChassis.getAngle().plus(Rotation2d.kCW_90deg.times(Math.signum(segmant.getAngleBetweenRadius().getRadians())));
            Rotation2d fixedVelocityHeadingWithRatio = tanToCircleAngle.times(centerToChassis.getNorm() / PathConstants.MAX_ANGULAR_VELOCITY);
            double velocity = driveTrapzoid.calculate(centerToChassis.getNorm(), chassisPoseAsVector.getNorm(), 0);
            if(Math.abs(chassisPoseAsVector.getNorm() - PathConstants.MAX_VELOCITY) < 0.1) velocity = PathConstants.MAX_VELOCITY;
            calculatedVel = new Translation2d(velocity, fixedVelocityHeadingWithRatio);
        }

        double angleError = MathUtil.angleModulus(CurrentSegmant.getEndPose().getAngle().getRadians() - currentPose.getRotation().getRadians());
        double omega = rotationPid.calculate(angleError, currentVelocity.omegaRadiansPerSecond);
        return new ChassisSpeeds(calculatedVel.getX(), calculatedVel.getY(), 0);
    }

}
