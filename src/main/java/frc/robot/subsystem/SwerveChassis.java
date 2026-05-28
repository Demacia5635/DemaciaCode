package frc.robot.subsystem;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
 
/**
 * Interface for the swerve drive subsystem.
 * Implement this in your robot's SwerveChassis class.
 */
public interface SwerveChassis extends Subsystem {
    /** Returns the current robot pose from odometry. */
    Pose2d getPose();
 
    /** Returns the current robot-relative chassis speeds. */
    ChassisSpeeds getChassisSpeedsRobotRel();
 
    /** Applies the given field-relative chassis speeds to the drivetrain. */
    void setVelocities(ChassisSpeeds speeds);
 
    /** Stops the drivetrain (called when the command ends). */
    void stop();
}