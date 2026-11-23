package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** nextVelocity(distance, currentVelocity, endVelocity) with maxVelocity 4 m/s and maxAccel 3 m/s^2. */
class DemaciaTrapezoidTest {

    private static final double EPS = 1e-9;
    private final DemaciaTrapezoid trapezoid = new DemaciaTrapezoid(4, 3);

    @Test
    void acceleratesByMaxAccelTimesDt() {
        assertEquals(1 + 3 * PathConstants.DT, trapezoid.nextVelocity(10, 1, 0), EPS);
    }

    @Test
    void accelerationIsCappedAtMaxVelocity() {
        assertEquals(4, trapezoid.nextVelocity(10, 3.99, 0), EPS);
    }

    @Test
    void deceleratesTowardsEndVelocity() {
        // vmax = sqrt((2*3*1 + 9 + 1) / 2) = 2.83 < 3, decel time = 2 * 1 / (3 + 1) = 0.5 s
        assertEquals(3 - (3 - 1) * PathConstants.DT / 0.5, trapezoid.nextVelocity(1, 3, 1), EPS);
    }

    @Test
    void shortDecelReturnsEndVelocity() {
        // decel time = 2 * 0.05 / 3 = 0.033 s, below 0.08 s
        assertEquals(1, trapezoid.nextVelocity(0.05, 2, 1), EPS);
    }

    @Test
    void decelTimeOfExactly80msStillDecelerates() {
        // decel time = 2 * 0.08 / 2 = 0.08 s, not below 0.08, so the normal decel step (oldPath)
        assertEquals(2 - 2 * PathConstants.DT / 0.08, trapezoid.nextVelocity(0.08, 2, 0), EPS);
    }

    @Test
    void stoppedAtTheEndStaysStoppedAndIsNotNaN() {
        // distance, speed and end speed all 0: the 0/0 guard must give the end speed, not NaN or a step up
        assertEquals(0, trapezoid.nextVelocity(0, 0, 0), 0);
    }
}
