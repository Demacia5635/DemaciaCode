package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.BaseSegment;

class TrajectoryBuilderTest {

    private static final double EPS = 1e-9;

    @BeforeAll
    static void initHal() {
        HAL.initialize(500, 0);
    }

    private static Waypoint wp(double x, double y) {
        return new Waypoint(x, y, new Rotation2d());
    }

    @Test
    void duplicateWaypointsThrow() {
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0), wp(3, 0), wp(3, 0), wp(3, 3))));
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0), wp(0, 0.005), wp(2, 0))));
    }

    @Test
    void cornerSpeedLimitsArcAndIncomingLine() {
        PathConfig config = new PathConfig().withMaxSpeed(3);
        List<Waypoint> points = List.of(wp(0, 0), wp(10, 0).withMaxCornerSpeed(2), wp(10, 10));

        List<BaseSegment> segments = TrajectoryBuilder.build(points, config).getSegments();

        assertEquals(3, segments.size());
        assertTrue(segments.get(0) instanceof LineSegment);
        assertTrue(segments.get(1) instanceof ArcSegment);
        assertTrue(segments.get(2) instanceof LineSegment);
        assertEquals(3, segments.get(0).getMaxVelocity(), EPS);
        // oldPath rule: r = v^2 / maxLateralAccel = 4/3 m (below 0.25 * 10 m), so the arc speed stays 2
        assertEquals(2, segments.get(1).getMaxVelocity(), EPS);
        assertEquals(3, segments.get(2).getMaxVelocity(), EPS);
        assertEquals(4.0 / config.maxLateralAccel, ((ArcSegment) segments.get(1)).getRadius(), EPS);
        assertEquals(2, segments.get(0).getEndVelocity(), EPS);
    }

    @Test
    void shortLegAheadLimitsPreviousEndVelocity() {
        PathConfig config = new PathConfig().withMaxAccel(2);
        double shortLeg = 0.25;
        // the stop point right after the short leg forces the leg before it to be able to brake
        List<Waypoint> points = List.of(wp(0, 0), wp(10, 0), wp(10 + shortLeg, 0).withMaxCornerSpeed(0), wp(20, 0));

        List<BaseSegment> segments = TrajectoryBuilder.build(points, config).getSegments();

        assertEquals(3, segments.size());
        assertEquals(shortLeg, segments.get(1).getLength(), EPS);
        assertEquals(0, segments.get(1).getEndVelocity(), EPS);
        double reachable = Math.sqrt(0 + 2 * config.maxAccel * shortLeg);
        assertEquals(reachable, segments.get(0).getEndVelocity(), EPS);
    }

    @Test
    void configEditedAfterBuildDoesNotChangeTrajectory() {
        PathConfig config = new PathConfig().withMaxSpeed(3);
        DemaciaTrajectory trajectory = TrajectoryBuilder.build(List.of(wp(0, 0), wp(5, 0), wp(5, 5)), config);
        double before = trajectory.getSegments().get(0).getMaxVelocity();

        config.withMaxSpeed(1).withMaxLateralAccel(0.5).withMaxAccel(0.5);

        assertEquals(before, trajectory.getSegments().get(0).getMaxVelocity(), EPS);
        assertEquals(3, before, EPS);
    }

    @Test
    void cornerSpeedAboveMaxSpeedIsCapped() {
        PathConfig config = new PathConfig().withMaxSpeed(2);
        List<BaseSegment> segments = TrajectoryBuilder
                .build(List.of(wp(0, 0), wp(10, 0).withMaxCornerSpeed(9), wp(10, 10)), config).getSegments();
        for (BaseSegment s : segments) {
            assertTrue(s.getMaxVelocity() <= 2 + EPS);
        }
    }

    @Test
    void invalidCapsThrow() {
        Waypoint mid = wp(5, 0);
        Waypoint end = wp(5, 5);
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0).withMaxCornerSpeed(1), mid, end)));
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0), mid, end.withMaxCornerSpeed(1))));
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0), mid.withMaxCornerSpeed(-1), end)));
        assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0), mid.withMaxCornerSpeed(Double.NaN), end)));
    }

    @Test
    void buildWithoutConfigEqualsDefaultConfig() {
        List<Waypoint> points = List.of(wp(0, 0), wp(5, 0), wp(5, 5).withMaxCornerSpeed(1), wp(0, 5));
        List<BaseSegment> a = TrajectoryBuilder.build(points).getSegments();
        List<BaseSegment> b = TrajectoryBuilder.build(points, new PathConfig()).getSegments();

        assertEquals(a.size(), b.size());
        for (int i = 0; i < a.size(); i++) {
            assertEquals(a.get(i).getClass(), b.get(i).getClass());
            assertEquals(a.get(i).getEndPose(), b.get(i).getEndPose());
            assertEquals(a.get(i).getEndVelocity(), b.get(i).getEndVelocity(), EPS);
            assertEquals(a.get(i).getMaxVelocity(), b.get(i).getMaxVelocity(), EPS);
            assertEquals(a.get(i).getLength(), b.get(i).getLength(), EPS);
        }
    }

    @Test
    void fewerThanTwoPointsGivesEmptyTrajectory() {
        assertTrue(TrajectoryBuilder.build(List.of()).getSegments().isEmpty());
        assertTrue(TrajectoryBuilder.build(List.of(wp(1, 1))).getSegments().isEmpty());
    }

    @Test
    void segmentListIsUnmodifiable() {
        List<BaseSegment> segments = TrajectoryBuilder.build(List.of(wp(0, 0), wp(1, 0))).getSegments();
        assertThrows(UnsupportedOperationException.class, segments::clear);
    }

    @Test
    void invalidConfigThrows() {
        List<Waypoint> points = List.of(wp(0, 0), wp(5, 0));
        List<java.util.function.Consumer<PathConfig>> breakers = List.of(
                c -> c.maxSpeed = 0, c -> c.maxAccel = -1, c -> c.maxLateralAccel = Double.NaN,
                c -> c.maxAngularSpeed = Double.POSITIVE_INFINITY, c -> c.maxAngularAccel = 0,
                c -> c.maxPositionThreshold = -0.1, c -> c.finalPositionThreshold = 0,
                c -> c.passedPointAngle = Double.NaN, c -> c.finalHeadingTolerance = 0);
        for (int i = 0; i < breakers.size(); i++) {
            PathConfig config = new PathConfig();
            breakers.get(i).accept(config);
            assertThrows(IllegalArgumentException.class, () -> TrajectoryBuilder.build(points, config));
        }
        TrajectoryBuilder.build(points, new PathConfig()); // the defaults are fine
    }

    @Test
    void arcRadiusUsesLateralAccelNotAccel() {
        PathConfig config = new PathConfig().withMaxAccel(1).withMaxLateralAccel(5);
        List<BaseSegment> segments = TrajectoryBuilder
                .build(List.of(wp(0, 0), wp(10, 0).withMaxCornerSpeed(3), wp(10, 10)), config).getSegments();

        ArcSegment arc = (ArcSegment) segments.get(1);
        assertEquals(9.0 / 5, arc.getRadius(), EPS); // 1.8 m, below 0.25 * 10 m
        assertEquals(3, arc.getMaxVelocity(), EPS);
    }

    @Test
    void brakingPassPropagatesBackwardOverSeveralSegments() {
        // collinear points have no arcs; the stop at (5.4, 0) must slow down both short legs before it
        PathConfig config = new PathConfig();
        List<BaseSegment> segments = TrajectoryBuilder.build(
                List.of(wp(0, 0), wp(5, 0), wp(5.2, 0), wp(5.4, 0).withMaxCornerSpeed(0), wp(10, 0)), config).getSegments();

        assertEquals(4, segments.size());
        assertEquals(0, segments.get(2).getEndVelocity(), EPS);
        assertEquals(Math.sqrt(2 * config.maxAccel * 0.2), segments.get(1).getEndVelocity(), EPS);
        assertEquals(Math.sqrt(2 * config.maxAccel * 0.4), segments.get(0).getEndVelocity(), EPS);
    }

    @Test
    void brakingPassUsesTheRealArcLength() {
        // low maxAccel so braking for the stop at (10, 2) limits both the arc and the line before it
        PathConfig config = new PathConfig().withMaxAccel(0.1);
        List<BaseSegment> segments = TrajectoryBuilder.build(
                List.of(wp(0, 0), wp(10, 0), wp(10, 2).withMaxCornerSpeed(0), wp(10, 10)), config).getSegments();

        assertEquals(4, segments.size());
        BaseSegment line = segments.get(0);
        ArcSegment arc = (ArcSegment) segments.get(1);
        BaseSegment toStop = segments.get(2);
        assertEquals(0.5, arc.getRadius(), EPS); // 0.25 * 2 m leg
        assertEquals(0, toStop.getEndVelocity(), EPS);
        assertEquals(Math.sqrt(2 * 0.1 * toStop.getLength()), arc.getEndVelocity(), EPS);
        assertEquals(Math.sqrt(arc.getEndVelocity() * arc.getEndVelocity() + 2 * 0.1 * arc.getLength()),
                line.getEndVelocity(), EPS);
    }

    @Test
    void cornerJustAboveOneDegreeGetsAnArc() {
        double turn = Math.toRadians(1.5);
        List<BaseSegment> withArc = TrajectoryBuilder
                .build(List.of(wp(0, 0), wp(5, 0), wp(10, 5 * Math.tan(turn)))).getSegments();
        assertEquals(3, withArc.size());
        assertTrue(withArc.get(1) instanceof ArcSegment);

        double small = Math.toRadians(0.5);
        assertEquals(2, TrajectoryBuilder.build(List.of(wp(0, 0), wp(5, 0), wp(10, 5 * Math.tan(small)))).getSegments().size());
    }

    @Test
    void finalPositionThresholdAppliesOnlyAtStopPoints() {
        PathConfig config = new PathConfig().withFinalPositionThreshold(0.3); // maxPositionThreshold stays 0.1

        // the last line ends at a stop
        BaseSegment last = TrajectoryBuilder.build(List.of(wp(0, 0), wp(5, 0)), config).getSegments().get(0);
        assertTrue(last.isFinished(new Translation2d(4.75, 0))); // 0.25 m away
        assertTrue(!last.isFinished(new Translation2d(4.65, 0))); // 0.35 m away

        // a stop point in the middle
        BaseSegment toStop = TrajectoryBuilder.build(List.of(wp(0, 0), wp(5, 0).withMaxCornerSpeed(0), wp(5, 5)), config)
                .getSegments().get(0);
        assertEquals(0, toStop.getEndVelocity(), EPS);
        assertTrue(toStop.isFinished(new Translation2d(4.75, 0)));
        assertTrue(!toStop.isFinished(new Translation2d(4.65, 0)));

        // a line that does not end at a stop uses the smaller threshold, unless the robot passed its end
        BaseSegment moving = TrajectoryBuilder.build(List.of(wp(0, 0), wp(10, 0), wp(10, 10)), config).getSegments().get(0);
        assertTrue(moving.getEndVelocity() > 0);
        // the line starts at (0, 0) and is slightly tilted to touch the corner circle
        Translation2d end = moving.getEndPose().getTranslation();
        Translation2d along = new Translation2d(1, end.getAngle());
        assertTrue(!moving.isFinished(end.minus(along.times(0.25))));
        assertTrue(moving.isFinished(end.minus(along.times(0.05))));
        assertTrue(moving.isFinished(end.plus(along.times(0.25)))); // passed
    }
}
