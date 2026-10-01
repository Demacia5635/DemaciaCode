package frc.demacia.swervePacgeWpilib.wpilibPoseEstimator;

import java.security.PublicKey;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

class wpilibPoseEstimator{
    private PoseEstimator poseEstimator;

    public wpilibPoseEstimator(SwerveDriveKinematics kinematics,SwerveDriveOdometry odometry, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs){
        poseEstimator = new PoseEstimator<>(kinematics, odometry, stateStdDevs, visionMeasurementStdDevs);
    }

    public Pose2d getPose2d(){
        return poseEstimator.getEstimatedPosition();
    }

    
}