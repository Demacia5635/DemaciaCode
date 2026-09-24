package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ChassisCommon {
    public static Pose2d currentRobotPose = Pose2d.kZero;
    public static Pose2d futureRobotPose = Pose2d.kZero;
    public static ChassisSpeeds robotRelSpeeds = new ChassisSpeeds();
    public static ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds();
    public static SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };
    public static Rotation2d chassisAngle = Rotation2d.kZero;
    public static ChassisSpeeds fieldRelAccel = new ChassisSpeeds();
    public static ChassisSpeeds fieldRelFutureSpeeds = new ChassisSpeeds();
}
