package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ChassisCommon {
    public static Pose2d currentRobotPose = Pose2d.kZero;
    public static Pose2d futureRobotPose = Pose2d.kZero;
    public static ChassisSpeeds robotRelSpeeds = new ChassisSpeeds();
    /** Current (measured) field-relative velocity. */
    public static ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds();
    /** Wanted (commanded) field-relative velocity. */
    public static ChassisSpeeds wantedFieldRelSpeeds = new ChassisSpeeds();
    public static SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };
    public static Rotation2d chassisAngle = Rotation2d.kZero;
    /** Current (measured) field-relative acceleration, from the change in velocity over the last cycle. */
    public static ChassisSpeeds fieldRelAccel = new ChassisSpeeds();
    /** Field-relative acceleration measured in the previous cycle. */
    public static ChassisSpeeds prevFieldRelAccel = new ChassisSpeeds();
    /** Field-relative acceleration needed to reach the wanted velocity within one cycle. */
    public static ChassisSpeeds wantedFieldRelAccel = new ChassisSpeeds();
    public static ChassisSpeeds fieldRelFutureSpeeds = new ChassisSpeeds();
}
