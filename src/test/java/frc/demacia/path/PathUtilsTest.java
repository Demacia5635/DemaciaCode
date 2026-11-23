package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** The math helpers in PathUtils, with hand-made numbers. */
class PathUtilsTest {

    private static final double EPS = 1e-9;

    private static void assertPoint(double x, double y, Translation2d actual) {
        assertEquals(x, actual.getX(), EPS, "x");
        assertEquals(y, actual.getY(), EPS, "y");
    }

    // ---------------------------------------------------------------- allNumbersAreFinite

    @Test
    void normalNumbersAreFinite() {
        Pose2d pose = new Pose2d(1, 2, Rotation2d.fromDegrees(30));
        ChassisSpeeds speeds = new ChassisSpeeds(1, -1, 0.5);
        assertTrue(PathUtils.allNumbersAreFinite(pose, speeds));
    }

    @Test
    void nanOrInfinityInAnyPoseNumberIsNotFinite() {
        ChassisSpeeds speeds = new ChassisSpeeds();
        assertFalse(PathUtils.allNumbersAreFinite(new Pose2d(Double.NaN, 0, new Rotation2d()), speeds));
        assertFalse(PathUtils.allNumbersAreFinite(new Pose2d(0, Double.POSITIVE_INFINITY, new Rotation2d()), speeds));
        assertFalse(PathUtils.allNumbersAreFinite(new Pose2d(0, 0, new Rotation2d(Double.NaN)), speeds));
    }

    @Test
    void nanOrInfinityInAnySpeedIsNotFinite() {
        Pose2d pose = new Pose2d();
        assertFalse(PathUtils.allNumbersAreFinite(pose, new ChassisSpeeds(Double.NaN, 0, 0)));
        assertFalse(PathUtils.allNumbersAreFinite(pose, new ChassisSpeeds(0, Double.NEGATIVE_INFINITY, 0)));
        assertFalse(PathUtils.allNumbersAreFinite(pose, new ChassisSpeeds(0, 0, Double.NaN)));
    }

    // ---------------------------------------------------------------- arcSweep

    @Test
    void leftQuarterTurnIsHalfPi() {
        // around (0, 0), from (1, 0) to (0, 1), counter-clockwise
        double sweep = PathUtils.arcSweep(new Translation2d(1, 0), new Translation2d(0, 1), new Translation2d(), true);
        assertEquals(Math.PI / 2, sweep, EPS);
    }

    @Test
    void rightTurnBetweenTheSamePointsGoesTheLongWayAround() {
        // same points as the left quarter turn, but clockwise: 3/4 of the circle
        double sweep = PathUtils.arcSweep(new Translation2d(1, 0), new Translation2d(0, 1), new Translation2d(), false);
        assertEquals(3 * Math.PI / 2, sweep, EPS);
    }

    @Test
    void sweepUsesTheCenterNotTheOrigin() {
        // half circle around (5, 5), from (6, 5) to (4, 5)
        double sweep = PathUtils.arcSweep(new Translation2d(6, 5), new Translation2d(4, 5), new Translation2d(5, 5), true);
        assertEquals(Math.PI, sweep, EPS);
    }

    @Test
    void sameStartAndEndIsAFullCircleNotZero() {
        // MathUtil.inputModulus maps 0 to 2 pi. The builder never makes a zero-length arc, so this is fine,
        // but this test fails if the behavior changes.
        double sweep = PathUtils.arcSweep(new Translation2d(1, 0), new Translation2d(1, 0), new Translation2d(), true);
        assertEquals(2 * Math.PI, sweep, EPS);
    }

    // ---------------------------------------------------------------- tangentPoints

    @Test
    void twoPlainPointsGiveThePointsThemselves() {
        List<Translation2d> leg = PathUtils.tangentPoints(new Translation2d(0, 0), 0, new Translation2d(3, 4), 0);
        assertEquals(2, leg.size());
        assertPoint(0, 0, leg.get(0));
        assertPoint(3, 4, leg.get(1));
    }

    @Test
    void twoLeftCirclesOfTheSameSizeGiveTheOuterLegOnTheRightSide() {
        // radius 1 circles at (0, 0) and (4, 0). Driving +x and turning left, the centers are on the
        // robot's left, so the leg runs along y = -1
        List<Translation2d> leg = PathUtils.tangentPoints(new Translation2d(0, 0), 1, new Translation2d(4, 0), 1);
        assertEquals(2, leg.size());
        assertPoint(0, -1, leg.get(0));
        assertPoint(4, -1, leg.get(1));
    }

    @Test
    void leftThenRightCircleGiveTheCrossingLeg() {
        // radius 1 at (0, 0) turning left, radius 1 at (4, 0) turning right: the leg crosses between them.
        // centers are 4 apart and the radii add to 2, so the leg is 30 deg off the center line
        List<Translation2d> leg = PathUtils.tangentPoints(new Translation2d(0, 0), 1, new Translation2d(4, 0), -1);
        assertEquals(2, leg.size());
        assertPoint(0.5, -Math.sqrt(3) / 2, leg.get(0));
        assertPoint(3.5, Math.sqrt(3) / 2, leg.get(1));
    }

    @Test
    void legTouchesBothCirclesAtARightAngle() {
        // different radii and a diagonal: check the geometry instead of exact numbers
        Translation2d center1 = new Translation2d(1, 2);
        Translation2d center2 = new Translation2d(6, -3);
        List<Translation2d> leg = PathUtils.tangentPoints(center1, 0.5, center2, -1.5);
        Translation2d direction = leg.get(1).minus(leg.get(0));

        assertEquals(0.5, leg.get(0).getDistance(center1), EPS, "touch point 1 is on circle 1");
        assertEquals(1.5, leg.get(1).getDistance(center2), EPS, "touch point 2 is on circle 2");
        // a tangent is perpendicular to the radius at the touch point: dot product = 0
        Translation2d radius1 = leg.get(0).minus(center1);
        Translation2d radius2 = leg.get(1).minus(center2);
        assertEquals(0, direction.getX() * radius1.getX() + direction.getY() * radius1.getY(), EPS);
        assertEquals(0, direction.getX() * radius2.getX() + direction.getY() * radius2.getY(), EPS);
    }

    @Test
    void circleInsideTheOtherHasNoLeg() {
        // a plain point 2 m from the center of a radius 3 circle is inside it
        List<Translation2d> leg = PathUtils.tangentPoints(new Translation2d(0, 0), 0, new Translation2d(2, 0), 3);
        assertTrue(leg.isEmpty());
    }

    @Test
    void sameCenterGivesBothCenters() {
        List<Translation2d> leg = PathUtils.tangentPoints(new Translation2d(1, 1), 1, new Translation2d(1, 1), 1);
        assertEquals(2, leg.size());
        assertPoint(1, 1, leg.get(0));
        assertPoint(1, 1, leg.get(1));
    }
}
