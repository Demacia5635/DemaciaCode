package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.util.List;
import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;

class PathConfigTest {

    private static double[] values(PathConfig c) {
        return new double[] { c.maxSpeed, c.maxAccel, c.maxLateralAccel, c.maxAngularSpeed, c.maxAngularAccel,
                c.maxPositionThreshold, c.finalPositionThreshold, c.passedPointAngle, c.finalHeadingTolerance };
    }

    @Test
    void defaults() {
        assertArrayEquals(new double[] { 4, 3, 3, 2, 0.6, 0.1, 0.1, Math.toRadians(60), Math.toRadians(5) },
                values(new PathConfig()), 1e-12);
    }

    @Test
    void eachSetterReturnsSameInstanceAndChangesOnlyItsField() {
        // same order as values()
        List<BiFunction<PathConfig, Double, PathConfig>> setters = List.of(
                PathConfig::withMaxSpeed,
                PathConfig::withMaxAccel,
                PathConfig::withMaxLateralAccel,
                PathConfig::withMaxAngularSpeed,
                PathConfig::withMaxAngularAccel,
                PathConfig::withMaxPositionThreshold,
                PathConfig::withFinalPositionThreshold,
                PathConfig::withPassedPointAngle,
                PathConfig::withFinalHeadingTolerance);

        for (int i = 0; i < setters.size(); i++) {
            PathConfig config = new PathConfig();
            double[] expected = values(config);
            expected[i] = 12.5;

            assertSame(config, setters.get(i).apply(config, 12.5), "setter " + i);
            assertArrayEquals(expected, values(config), 1e-12, "setter " + i);
        }
    }

    @Test
    void settersCanBeChained() {
        PathConfig config = new PathConfig().withMaxSpeed(1).withMaxAccel(2);
        assertEquals(1, config.maxSpeed);
        assertEquals(2, config.maxAccel);
    }

    @Test
    void defaultInstancesAreIndependent() {
        PathConfig a = new PathConfig();
        PathConfig b = new PathConfig();
        a.withMaxSpeed(1);
        assertNotEquals(a.maxSpeed, b.maxSpeed);
        assertEquals(4, b.maxSpeed);
    }
}
