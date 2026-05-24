package frc.robot.commend;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SwerveChassis;
import frc.robot.trajectory.DemaciaTrajectory;

import java.util.List;

import com.ctre.phoenix.motion.TrajectoryPoint;

/**
 * WPILib Command that follows a Demacia curved path.
 *
 * Usage example (in RobotContainer):
 *   List<TrajectoryPoint> points = List.of(
 *       new TrajectoryPoint(new Pose2d(0, 0, new Rotation2d(0)),   0,   3.0, 2.0),
 *       new TrajectoryPoint(new Pose2d(3, 2, new Rotation2d(0)),   1.5, 3.0, 2.0),
 *       new TrajectoryPoint(new Pose2d(6, 0, new Rotation2d(0)),   0,   3.0, 2.0)
 *   );
 *   new PathCommand(chassis, points, 0.5).schedule();
 */
public class PathCommand extends Command {

    private final SwerveChassis chassis;
    private final DemaciaTrajectory trajectory;
    private final List<TrajectoryPoint> points;
    private final double radius;

    /**
     * @param chassis  the swerve drive subsystem
     * @param points   ordered list of waypoints defining the path
     * @param radius   uniform turn radius for all curves (meters)
     */
    public PathCommand(SwerveChassis chassis, List<TrajectoryPoint> points, double radius) {
        this.chassis    = chassis;
        this.points     = points;
        this.radius     = radius;
        this.trajectory = new DemaciaTrajectory();
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        // Build the leg list once when the command starts
        trajectory.build(points, radius);
    }

    @Override
    public void execute() {
        // Compute and apply required speeds every cycle
        ChassisSpeeds needSpeed = trajectory.calculateSpeeds(
                chassis.getChassisSpeedsRobotRel(),
                chassis.getPose());
        chassis.setVelocities(needSpeed);
    }

    @Override
    public boolean isFinished() {
        return trajectory.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
