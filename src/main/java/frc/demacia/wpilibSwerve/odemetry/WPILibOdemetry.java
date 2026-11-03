package frc.demacia.wpilibSwerve.odemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class WPILibOdemetry{
    private SwerveDriveOdometry odometry;

    public WPILibOdemetry (SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePosition){
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, modulePosition);
    }

    public void resetPose(Pose2d pose){
        odometry.resetPose(pose);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void updatePose(Rotation2d gyroAngle,SwerveModulePosition[] modulePositions){
        odometry.update(gyroAngle, modulePositions);
    }

    public SwerveDriveOdometry getOdometry(){
        return odometry;
    }
}