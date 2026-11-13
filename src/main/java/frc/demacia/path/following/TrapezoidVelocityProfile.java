package frc.demacia.path.following;

import frc.demacia.path.constants.PathConstants;

/** Decides the next speed along a segment so the robot accelerates, cruises and brakes in a trapezoid shape. */
public class TrapezoidVelocityProfile {

    /** If braking would take less than this, jump straight to the end velocity. */
    private static final double MIN_BRAKING_TIME_SECONDS = 0.08;

    private final double maxVelocityMps;
    private final double maxAccelerationMps2;
    private final double maxVelocityChangePerCycle;

    public TrapezoidVelocityProfile(double maxVelocityMps, double maxAccelerationMps2) {
        this.maxVelocityMps = maxVelocityMps;
        this.maxAccelerationMps2 = maxAccelerationMps2;
        this.maxVelocityChangePerCycle = maxAccelerationMps2 * PathConstants.CONTROL_LOOP_PERIOD_SECONDS;
    }

    /**
     * Next speed command: accelerate while it is still possible to reach {@code endVelocityMps}
     * at the end of the distance, otherwise brake smoothly.
     */
    public double calculateNextVelocity(double distanceToEndMeters, double currentVelocityMps, double endVelocityMps) {
        double maxReachableVelocity = Math.sqrt(
                (distanceToEndMeters * 2 * maxAccelerationMps2
                        + currentVelocityMps * currentVelocityMps
                        + endVelocityMps * endVelocityMps) / 2);

        if (maxReachableVelocity > currentVelocityMps) {
            // we can still accelerate
            return Math.min(currentVelocityMps + maxVelocityChangePerCycle, maxVelocityMps);
        }

        // we need to brake
        double brakingTimeSeconds = 2 * distanceToEndMeters / (currentVelocityMps + endVelocityMps);
        if (brakingTimeSeconds < MIN_BRAKING_TIME_SECONDS) {
            return endVelocityMps;
        }
        return currentVelocityMps
                - (currentVelocityMps - endVelocityMps) * PathConstants.CONTROL_LOOP_PERIOD_SECONDS / brakingTimeSeconds;
    }

    /**
     * Alternative step-by-step version based on stopping distance (currently not used by the follower).
     */
    public double calculateNextVelocityByStoppingDistance(double distanceToEndMeters, double currentVelocityMps,
            double endVelocityMps) {
        double velocityIfAccelerating = Math.min(currentVelocityMps + maxVelocityChangePerCycle, maxVelocityMps);
        double velocityIfBraking = Math.max(currentVelocityMps - maxVelocityChangePerCycle, endVelocityMps);

        // Must start braking now to reach the end velocity in time
        if (calculateStoppingDistance(currentVelocityMps, endVelocityMps) >= distanceToEndMeters) {
            return velocityIfBraking;
        }

        // Can accelerate and still have room to brake afterwards
        if (calculateStoppingDistance(velocityIfAccelerating, endVelocityMps) < distanceToEndMeters) {
            return velocityIfAccelerating;
        }

        // Otherwise cruise
        return currentVelocityMps;
    }

    /** Distance needed to brake from {@code velocityMps} down to {@code endVelocityMps}. */
    private double calculateStoppingDistance(double velocityMps, double endVelocityMps) {
        if (velocityMps <= endVelocityMps) {
            return 0;
        }
        return (velocityMps * velocityMps - endVelocityMps * endVelocityMps) / (2 * maxAccelerationMps2);
    }
}
