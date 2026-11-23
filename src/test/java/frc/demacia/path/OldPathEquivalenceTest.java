package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.EquivalenceRoutes.Route;
import frc.demacia.path.segments.BaseSegment;

/**
 * The new path code must give exactly the same output as the old path code did.
 * src/test/resources/path/oldpath-fixtures.csv was recorded once from the old code (oldPath, now deleted):
 * for every route and tracking value, the segment list and every cycle's inputs (pose, field speeds)
 * and outputs (speeds, isFinished). Here the same inputs go to the new code.
 * The routes are in EquivalenceRoutes.
 */
class OldPathEquivalenceTest {

    private static final double EPS = 1e-9;
    private static final String FIXTURES = "/path/oldpath-fixtures.csv";

    @BeforeAll
    static void initHal() {
        HAL.initialize(500, 0);
    }

    @Test
    void leftTurn() {
        check("leftTurn#0");
    }

    @Test
    void mirroredRightTurn() {
        check("mirroredRightTurn#0");
    }

    @Test
    void straightTwoPoints() {
        check("straightTwoPoints#0");
    }

    @Test
    void lateralOffsetStart() {
        check("lateralOffsetStart#0");
    }

    @Test
    void shortLegForcesArcRemoval() {
        check("shortLegForcesArcRemoval#0");
        check("shortLegForcesArcRemoval#1");
    }

    @Test
    void midPathStopPoint() {
        check("midPathStopPoint#0");
    }

    @Test
    void slowCornerAfterFastLine() {
        check("slowCornerAfterFastLine#0");
    }

    @Test
    void finalPositionReachedWithHeadingStillOff() {
        check("finalPositionReachedWithHeadingStillOff#0");
    }

    @Test
    void reversalOver179Degrees() {
        check("reversalOver179Degrees#0");
    }

    @Test
    void collinearPointsUnder1Degree() {
        check("collinearPointsUnder1Degree#0");
    }

    @Test
    void stopPointAtIndexOne() {
        check("stopPointAtIndexOne#0");
    }

    @Test
    void startWithHeadingErrorAndOmega() {
        check("startWithHeadingErrorAndOmega#0");
    }

    // ---------------------------------------------------------------------------------

    private void check(String routeId) {
        Route route = EquivalenceRoutes.get(routeId);
        List<String[]> fixture = readFixture();
        for (double tracking : EquivalenceRoutes.TRACKINGS) {
            replay(route, tracking, fixture);
        }
    }

    private void replay(Route route, double tracking, List<String[]> fixture) {
        List<Waypoint> points = new ArrayList<>();
        for (double[] r : route.rows()) {
            Waypoint w = new Waypoint(r[0], r[1], Rotation2d.fromDegrees(r[2]));
            points.add(r.length > 3 ? w.withMaxCornerSpeed(r[3]) : w);
        }
        DemaciaTrajectory trajectory = TrajectoryBuilder.build(points);
        List<BaseSegment> segments = trajectory.getSegments();

        List<String[]> segRows = rows(fixture, "SEG", route.id(), tracking);
        List<String[]> cycleRows = rows(fixture, "CYC", route.id(), tracking);
        String t = " (" + route.id() + ", tracking " + tracking + ")";

        // an arc was really removed when the route says so
        if (route.id().startsWith("shortLegForcesArcRemoval")) {
            assertTrue(segments.size() < route.maxSegments(),
                    "no arc removed: " + segments.size() + " segments, " + route.maxSegments() + " without removal" + t);
        }
        assertEquals(segRows.size(), segments.size(), "segment count" + t);
        for (int i = 0; i < segRows.size(); i++) {
            String[] o = segRows.get(i);
            BaseSegment n = segments.get(i);
            String s = "segment " + i + " " + t + " ";
            assertEquals(d(o, 4), n.getEndPose().getX(), EPS, s + "end x");
            assertEquals(d(o, 5), n.getEndPose().getY(), EPS, s + "end y");
            assertEquals(d(o, 6), n.getEndPose().getRotation().getRadians(), EPS, s + "end heading");
            assertEquals(d(o, 7), n.getEndVelocity(), EPS, s + "end velocity");
            assertEquals(d(o, 8), n.getMaxVelocity(), EPS, s + "max velocity");
            assertEquals(d(o, 9), n.getLength(), EPS, s + "length");
        }

        PathFollower follower = new PathFollower();
        follower.start(trajectory);
        assertTrue(!cycleRows.isEmpty(), "no cycles in the fixture" + t);
        for (String[] c : cycleRows) {
            int cycle = (int) d(c, 3);
            Pose2d pose = new Pose2d(d(c, 4), d(c, 5), new Rotation2d(d(c, 6)));
            ChassisSpeeds speeds = new ChassisSpeeds(d(c, 7), d(c, 8), d(c, 9));
            ChassisSpeeds actual = follower.calculate(pose, speeds);

            String where = " at cycle " + cycle + t;
            assertEquals(d(c, 10), actual.vxMetersPerSecond, EPS, "vx" + where);
            assertEquals(d(c, 11), actual.vyMetersPerSecond, EPS, "vy" + where);
            assertEquals(d(c, 12), actual.omegaRadiansPerSecond, EPS, "omega" + where);
            assertEquals(Boolean.parseBoolean(c[13]), follower.isFinished(), "isFinished" + where);
        }
    }

    // ---------------------------------------------------------------------------------

    private static double d(String[] row, int index) {
        return Double.parseDouble(row[index]);
    }

    private static List<String[]> rows(List<String[]> fixture, String kind, String routeId, double tracking) {
        List<String[]> result = new ArrayList<>();
        for (String[] row : fixture) {
            if (row[0].equals(kind) && row[1].equals(routeId) && Double.parseDouble(row[2]) == tracking) {
                result.add(row);
            }
        }
        return result;
    }

    private static List<String[]> readFixture() {
        List<String[]> rows = new ArrayList<>();
        try (InputStream in = OldPathEquivalenceTest.class.getResourceAsStream(FIXTURES)) {
            assertTrue(in != null, "missing " + FIXTURES);
            BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
            String line;
            while ((line = reader.readLine()) != null) {
                if (!line.isBlank() && !line.startsWith("#")) {
                    rows.add(line.split(","));
                }
            }
        } catch (IOException e) {
            throw new AssertionError(e);
        }
        return rows;
    }
}
