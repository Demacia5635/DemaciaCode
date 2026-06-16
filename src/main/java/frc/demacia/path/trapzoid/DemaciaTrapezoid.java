package frc.demacia.path.trapzoid;

import frc.demacia.utils.log.LogManager;

public class DemaciaTrapezoid {
    private final double maxVelocity;
    private final double maxAccel;
    private final double dt = 0.02;
    private final double maxDeltaV;

    public DemaciaTrapezoid(double maxVelocity, double maxAccel) {
        this.maxAccel = maxAccel;
        this.maxVelocity = maxVelocity;
        this.maxDeltaV = maxAccel * dt;
    }

    // Distance needed to decelerate from velocity v to finishVelocity
    private double stoppingDistance(double v, double finishVelocity) {
        if (v <= finishVelocity)
            return 0;
        return (v * v - finishVelocity * finishVelocity) / (2 * maxAccel);
    }

    public double nextVelocity(double distance, double currentVelocity, double endVelocity){
        double vmax = Math.sqrt((distance * 2 * maxAccel + currentVelocity*currentVelocity + endVelocity*endVelocity) / 2);
        LogManager.log("vmax: " + vmax + " currentVelocity: " + currentVelocity);
        if(vmax > currentVelocity) { // we can accelerate
            return Math.max(currentVelocity + maxAccel * 0.02, maxVelocity);
        } else { // we need to deccelerate
            double deaccelTime = 2 * distance / (currentVelocity + endVelocity);
            if(deaccelTime < 0.02) {
                return endVelocity;
            } else {
                return currentVelocity - (currentVelocity - endVelocity) * 0.02 / deaccelTime;
            }
        }
    }

    public double calculate(double distanceLeft, double currentVelocity, double finishVelocity) {
        double nextVelocityIfAccel = Math.min(currentVelocity + maxDeltaV, maxVelocity);
        double nextVelocityIfCoast = currentVelocity;
        double nextVelocityIfDecel = Math.max(currentVelocity - maxDeltaV, finishVelocity);

        // If we need to start braking now to reach finishVelocity in time, decelerate
        if (stoppingDistance(currentVelocity, finishVelocity) >= distanceLeft) {
            return nextVelocityIfDecel;
        }

        // If we can accelerate and still have room to stop afterward, accelerate
        if (nextVelocityIfAccel <= maxVelocity
                && stoppingDistance(nextVelocityIfAccel, finishVelocity) < distanceLeft) {
            return nextVelocityIfAccel;
        }

        // Otherwise cruise
        return nextVelocityIfCoast;
    }


}