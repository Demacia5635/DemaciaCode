package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;

/** LineSegment and ArcSegment on their own, with hand-made geometry. */
class SegmentTest {

    private static final double EPS = 1e-9;

    /** Line from (0, 0) to (5, 0) that does not end at a stop. */
    private static LineSegment movingLine(PathConfig config) {
        return new LineSegment(new Translation2d(), new Pose2d(5, 0, new Rotation2d()), 1, 4, config);
    }

    /** Left (counter-clockwise) quarter circle around (0, 0), radius 1, from (1, 0) to (0, 1). */
    private static ArcSegment quarterArc(PathConfig config) {
        return new ArcSegment(new Translation2d(1, 0), new Pose2d(0, 1, new Rotation2d()), new Translation2d(), true, 1,
                1, 1, config);
    }

    private static Translation2d onUnitCircle(double degrees) {
        return new Translation2d(1, Rotation2d.fromDegrees(degrees));
    }

    // ---------------------------------------------------------------- line

    @Test
    void lineCountsAsPassedOnBothSidesOfTheLine() {
        LineSegment line = movingLine(new PathConfig());
        // 0.58 m past the end, 31 deg off the line on either side
        assertTrue(line.isFinished(new Translation2d(5.5, 0.3)));
        assertTrue(line.isFinished(new Translation2d(5.5, -0.3)));
        // before the end, 45 deg off on either side: not passed
        assertFalse(line.isFinished(new Translation2d(4, 1)));
        assertFalse(line.isFinished(new Translation2d(4, -1)));
    }

    @Test
    void stopLineIsNotFinishedAtExactlyTheThreshold() {
        PathConfig config = new PathConfig().withFinalPositionThreshold(0.25);
        LineSegment stop = new LineSegment(new Translation2d(), new Pose2d(5, 0, new Rotation2d()), 0, 4, config);
        assertFalse(stop.isFinished(new Translation2d(4.75, 0))); // 0.25 m, strict < like oldPath
        assertTrue(stop.isFinished(new Translation2d(4.76, 0)));
    }

    @Test
    void lineDirectionAtItsEndIsTheLineDirection() {
        LineSegment up = new LineSegment(new Translation2d(), new Pose2d(0, 2, new Rotation2d()), 0, 4, new PathConfig());
        assertEquals(Math.PI / 2, up.getDirection(new Translation2d(0, 2), 0).getRadians(), EPS);
    }

    @Test
    void robotExactlyBesideTheEndStillUsesTheMirrorRule() {
        // to the end is exactly 90 deg off the line: not "passed" (strict >), so the mirror rule gives 180 deg (oldPath)
        LineSegment line = new LineSegment(new Translation2d(), new Pose2d(2, 0, new Rotation2d()), 1, 4, new PathConfig());
        assertEquals(-1, line.getDirection(new Translation2d(2, 1), 0).getCos(), EPS);
    }

    @Test
    void lineThresholdsAreCopiedNotReadFromTheConfigLater() {
        PathConfig config = new PathConfig().withMaxPositionThreshold(0.2).withFinalPositionThreshold(0.3);
        LineSegment moving = movingLine(config);
        LineSegment stop = new LineSegment(new Translation2d(), new Pose2d(5, 0, new Rotation2d()), 0, 4, config);

        config.withMaxPositionThreshold(0.01).withFinalPositionThreshold(0.01);

        assertTrue(moving.isFinished(new Translation2d(4.85, 0)));
        assertTrue(stop.isFinished(new Translation2d(4.75, 0)));
    }

    // ---------------------------------------------------------------- arc

    @Test
    void arcUsesMaxPositionThresholdNotFinalThreshold() {
        Translation2d nearEnd = onUnitCircle(80); // 0.17 m before the end, still on the arc
        assertTrue(quarterArc(new PathConfig().withMaxPositionThreshold(0.3)).isFinished(nearEnd));
        assertFalse(quarterArc(new PathConfig().withFinalPositionThreshold(0.3)).isFinished(nearEnd));
    }

    @Test
    void arcThresholdIsCopiedNotReadFromTheConfigLater() {
        PathConfig config = new PathConfig().withMaxPositionThreshold(0.3);
        ArcSegment arc = quarterArc(config);
        config.withMaxPositionThreshold(0.01);
        assertTrue(arc.isFinished(onUnitCircle(80)));
    }

    @Test
    void arcProfileUsesMaxAccelNotLateralAccel() {
        ArcSegment arc = new ArcSegment(new Translation2d(1, 0), new Pose2d(0, 1, new Rotation2d()), new Translation2d(),
                true, 1, 1, 4, new PathConfig().withMaxAccel(1).withMaxLateralAccel(5));
        assertEquals(1 * PathConstants.DT, arc.getVelocity(new Translation2d(1, 0), new Translation2d()).getNorm(), EPS);
    }

    @Test
    void arcSpeedNeverAboveItsMaxEvenWhenEnteringTooFast() {
        // 0.5 rad left, entering at 2.5 m/s, arc max 1 m/s: the decel step alone gives 2.39 m/s
        ArcSegment arc = quarterArc(new PathConfig());
        Translation2d position = new Translation2d(1, new Rotation2d(Math.PI / 2 - 0.5));
        assertEquals(1, arc.getVelocity(position, new Translation2d(0, 2.5)).getNorm(), EPS);
    }

    @Test
    void arcDistanceLeftIsZeroPastTheEnd() {
        ArcSegment arc = quarterArc(new PathConfig());
        assertEquals(0, arc.getDistanceLeft(onUnitCircle(100)), 0);
        assertEquals(Math.toRadians(30), arc.getDistanceLeft(onUnitCircle(60)), EPS);
        assertTrue(arc.isFinished(onUnitCircle(100)));
    }
}
