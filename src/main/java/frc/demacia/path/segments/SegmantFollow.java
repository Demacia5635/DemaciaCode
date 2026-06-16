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

        Translation2d calculatedVel;
        if(PathUtils.isLineSegment(CurrentSegmant)){
            LogManager.log("line segment");
            LineSegment segmant = (LineSegment) CurrentSegmant;
            // Translation2d VectorToFinish = segmant.getEndPose().minus(chassisPoseAsVector);
            // double vel = driveTrapzoid.nextVelocity(VectorToFinish.getNorm(), currentVelVector.getNorm(), finalVel);
            // Rotation2d HadingError = segmant.getStartToEndVector().getAngle().minus(currentPose.getTranslation().getAngle());
            // Rotation2d fixeHading = VectorToFinish.getAngle().minus(HadingError.times(2));
            // LogManager.log("vel: " + vel + " fixedVel: " + fixeHading);
            // calculatedVel = new Translation2d(vel, fixeHading);
            // LineSegment segment = (LineSegment) currentSegment;
            double getDesteans = segmant.getTranslation().getNorm() - chassisPoseAsVector.minus(segmant.getStartPose()).getNorm();
            double vel = driveTrapzoid.nextVelocity(getDesteans, currentVelVector.getNorm(), segmant.getFinalVelocity());
            double heading = (2 * chassisPoseAsVector.getAngle().getRadians()) + segmant.getTranslation().getAngle().getRadians();
            calculatedVel = new Translation2d(vel, new Rotation2d(heading));
        }
        
        else{
            LogManager.log("not line segment");
            ArcSegment segmant = (ArcSegment) CurrentSegmant;
            Translation2d centerToChassis = chassisPoseAsVector.minus(segmant.getCenter());
            Rotation2d tanToCircleAngle = centerToChassis.getAngle().plus(Rotation2d.kCW_90deg.times(Math.signum(segmant.getAngleBetweenRadius().getRadians())));
            Rotation2d fixedVelocityHeadingWithRatio = tanToCircleAngle.times(centerToChassis.getNorm() / PathConstants.MAX_ANGULAR_VELOCITY);
            double velocity = driveTrapzoid.calculate(centerToChassis.getNorm(), chassisPoseAsVector.getNorm(), 0);
            if(Math.abs(chassisPoseAsVector.getNorm() - PathConstants.MAX_VELOCITY) < 0.1) velocity = PathConstants.MAX_VELOCITY;
            LogManager.log("velocity: " + velocity + " fixedVelocityHeadingWithRatio: " + fixedVelocityHeadingWithRatio);
            calculatedVel = new Translation2d(velocity, fixedVelocityHeadingWithRatio);
        }

        double angleError = MathUtil.angleModulus(CurrentSegmant.getEndPose().getAngle().getRadians() - currentPose.getRotation().getRadians());
        double omega = rotationPid.calculate(angleError, currentVelocity.omegaRadiansPerSecond);
        LogManager.log("CurrentSegmant: " + CurrentSegmant + " currentPose: " + currentPose + " currentVelocity: " + currentVelocity  + " chassis wanted speed: " + new ChassisSpeeds(calculatedVel.getX(), calculatedVel.getY(), omega));
        return new ChassisSpeeds(calculatedVel.getX(), calculatedVel.getY(), 0 /*omega*/);
    }

}
