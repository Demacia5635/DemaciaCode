package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A single waypoint in the path.
 * Holds the robot's pose, the desired velocity at this point,
 * and the motion constraints for the leg leading into it.
 */
public class TrajectoryPoint {
    public final Pose2d pose;
    public final double velocity;         // desired velocity at this waypoint (m/s)
    public final double maxVelocity;      // max velocity allowed on the leg to this point (m/s)
    public final double maxAcceleration;  // max acceleration allowed on the leg to this point (m/s²)

    public TrajectoryPoint(Pose2d pose, double velocity,
                           double maxVelocity, double maxAcceleration) {
        this.pose            = pose;
        this.velocity        = velocity;
        this.maxVelocity     = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}
