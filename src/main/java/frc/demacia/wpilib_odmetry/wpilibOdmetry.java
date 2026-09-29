package frc.demacia.wpilib_odmetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

class wpilivOdmetry{
    private SwerveDriveOdometry odometry;

    public wpilivOdmetry (SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition modulePosition){
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, modulePosition);
    }
}