package frc.robot.trajectory;

/**
 * Return value from leg tracking calculations.
 * Contains the desired velocity, movement heading, and remaining distance.
 */
public class CalculateResult {
    public final double velocity;       // desired scalar velocity (m/s)
    public final double heading;        // movement direction (radians, field-relative)
    public final double distanceLeft;   // remaining distance to end of leg (m)

    public CalculateResult(double velocity, double heading, double distanceLeft) {
        this.velocity     = velocity;
        this.heading      = heading;
        this.distanceLeft = distanceLeft;
    }
}
