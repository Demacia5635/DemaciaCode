package frc.demacia.path;

/**
 * Decides how fast to drive along one segment: speed up, cruise, then slow down.
 * Each segment owns one. Its speed graph looks like a trapezoid.
 */
public class DemaciaTrapezoid {
    private final double maxVelocity;
    private final double maxAccel;

    /**
     * @param maxVelocity top speed on this segment, in m/s
     * @param maxAccel speed-up and slow-down rate, in m/s^2
     */
    public DemaciaTrapezoid(double maxVelocity, double maxAccel) {
        this.maxAccel = maxAccel;
        this.maxVelocity = maxVelocity;
    }

    /**
     * Picks the speed to ask for during the next 20 ms.
     *
     * @param distance distance left on the segment, in m
     * @param currentVelocity measured speed of the robot, in m/s
     * @param endVelocity speed the robot should have at the end of the segment, in m/s
     * @return the speed to drive, in m/s (never above the max speed when speeding up)
     */
    public double nextVelocity(double distance, double currentVelocity, double endVelocity) {
        // Fastest speed we could reach and still be able to brake to endVelocity in the distance left.
        // This is the top of a "triangle" speed graph: speed up at maxAccel, then slow down at maxAccel.
        double maxVelocityWeCanReach = Math.sqrt(
                (distance * 2 * maxAccel + currentVelocity * currentVelocity + endVelocity * endVelocity) / 2);

        if (maxVelocityWeCanReach > currentVelocity) {
            // we have room: speed up by one step, but not above the top speed
            return Math.min(currentVelocity + maxAccel * PathConstants.DT, maxVelocity);
        }

        // we need to slow down
        // time to slow down evenly from currentVelocity to endVelocity over the distance left
        // (average speed is (current + end) / 2)
        double decelTime = 2 * distance / (currentVelocity + endVelocity);
        // The first check avoids dividing by zero (the robot is stopped and wants to stay stopped).
        // Under 0.08 s of braking is left, so a small step would be tiny: jump to the end speed.
        boolean almostThere = currentVelocity + endVelocity <= 0 || decelTime < 0.08;
        if (almostThere) {
            return endVelocity;
        }
        return currentVelocity - (currentVelocity - endVelocity) * PathConstants.DT / decelTime;
    }
}
