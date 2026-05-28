package frc.robot.trajectory;

/**
 * Trapezoidal velocity profile calculator.
 * Computes the desired velocity for the next cycle given current speed,
 * target end speed, motion limits, and remaining distance.
 */
public class Trapezoid {

    /**
     * @param currentV   current velocity (m/s)
     * @param targetEndV desired velocity at the end of the leg (m/s)
     * @param maxV       maximum allowed velocity (m/s)
     * @param maxA       maximum allowed acceleration (m/s²)
     * @param distLeft   remaining distance to the end of the leg (m)
     * @return desired velocity for the next cycle (m/s)
     */
    public static double calculate(double currentV, double targetEndV,
                                   double maxV, double maxA, double distLeft) {
        // Maximum speed we can travel and still brake to targetEndV in distLeft
        // derived from: v² = targetEndV² + 2*a*d
        double brakingV = Math.sqrt(targetEndV * targetEndV + 2 * maxA * distLeft);

        // Desired speed is the minimum of the two constraints
        double wantedV = Math.min(maxV, brakingV);

        // Clamp acceleration / deceleration to maxA per cycle
        double dt = 0.02; // cycle time (seconds)
        if (wantedV > currentV) {
            return Math.min(wantedV, currentV + maxA * dt);
        } else {
            return Math.max(wantedV, currentV - maxA * dt);
        }
    }
}
