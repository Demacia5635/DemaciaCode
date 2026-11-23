package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class WaypointTest {

    @Test
    void defaultCapIsInfinite() {
        assertEquals(Double.POSITIVE_INFINITY, new Waypoint(1, 2, new Rotation2d()).getMaxCornerSpeed());
    }

    @Test
    void constructorsGiveSamePose() {
        Pose2d pose = new Pose2d(1, 2, Rotation2d.fromDegrees(30));
        assertEquals(pose, new Waypoint(pose));
        assertEquals(pose, new Waypoint(new Translation2d(1, 2), Rotation2d.fromDegrees(30)));
        assertEquals(pose, new Waypoint(1, 2, Rotation2d.fromDegrees(30)));
    }

    @Test
    void withMaxCornerSpeedReturnsNewWaypointAndKeepsOriginal() {
        Waypoint original = new Waypoint(1, 2, Rotation2d.fromDegrees(30));
        Waypoint slow = original.withMaxCornerSpeed(1.5);

        assertNotSame(original, slow);
        assertEquals(Double.POSITIVE_INFINITY, original.getMaxCornerSpeed());
        assertEquals(1.5, slow.getMaxCornerSpeed());
        assertEquals(original, slow); // same pose
        assertEquals(0, original.withMaxCornerSpeed(0).getMaxCornerSpeed());
    }
}
