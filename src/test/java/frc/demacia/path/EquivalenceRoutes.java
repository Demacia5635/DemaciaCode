package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Routes of the old-vs-new equivalence test. Each route is a table of rows
 * {x m, y m, heading deg} or {x, y, heading, max corner speed m/s}.
 * The id is "testName#index", it is the key in oldpath-fixtures.csv.
 */
final class EquivalenceRoutes {

    static final double[] TRACKINGS = { 1.0, 0.7 };

    record Route(String id, double[][] rows, double startDx, double startDy, double startHeadingDeg, double startOmega) {
        /** Interior points that get a corner arc before any arc is removed. */
        int candidateArcs() {
            int count = 0;
            for (int i = 1; i < rows.length - 1; i++) {
                Translation2d prev = new Translation2d(rows[i - 1][0], rows[i - 1][1]);
                Translation2d here = new Translation2d(rows[i][0], rows[i][1]);
                Translation2d next = new Translation2d(rows[i + 1][0], rows[i + 1][1]);
                double v = rows[i].length > 3 ? rows[i][3] : 4;
                double turn = Math.abs(MathUtil.angleModulus(
                        next.minus(here).getAngle().getRadians() - here.minus(prev).getAngle().getRadians()));
                double maxR = Math.min(prev.getDistance(here), here.getDistance(next)) * PathConstants.MAX_RADIUS_TO_LEG_RATIO;
                boolean reversal = turn > Math.PI - PathConstants.MIN_TURN_FOR_ARC;
                if (v == 0 || reversal || turn < PathConstants.MIN_TURN_FOR_ARC || maxR < 1e-6) {
                    continue;
                }
                count++;
            }
            return count;
        }

        /** Segment count when no arc is removed. */
        int maxSegments() {
            return rows.length - 1 + candidateArcs();
        }
    }

    static final List<Route> ALL = new ArrayList<>();

    private static void add(String name, double[][] rows, double dx, double dy, double headingDeg) {
        add(name, rows, dx, dy, headingDeg, 0);
    }

    private static void add(String name, double[][] rows, double dx, double dy, double headingDeg, double omega) {
        long index = ALL.stream().filter(r -> r.id().startsWith(name + "#")).count();
        ALL.add(new Route(name + "#" + index, rows, dx, dy, headingDeg, omega));
    }

    static {
        add("leftTurn", new double[][] { { 0, 0, 0 }, { 5, 0, 0 }, { 5, 5, 90 } }, 0, 0, 0);
        add("mirroredRightTurn", new double[][] { { 0, 0, 0 }, { 5, 0, 0 }, { 5, -5, -90 } }, 0, 0, 0);
        add("straightTwoPoints", new double[][] { { 0, 0, 0 }, { 6, 0, 0 } }, 0, 0, 0);
        add("lateralOffsetStart", new double[][] { { 0, 0, 0 }, { 5, 0, 0 }, { 5, 5, 90 } }, 0, 0.3, 0);
        // both of these lose one corner arc (found by search, checked by the segment count)
        add("shortLegForcesArcRemoval", new double[][] { { 7.6, 4.2, 0 }, { 3.1, 2.2, 90 }, { 0.9, 1.8, 180 }, { 7.9, 4.7, 0 } }, 0, 0, 0);
        add("shortLegForcesArcRemoval", new double[][] { { 0.8, 3.1, 0 }, { 6.0, 2.9, 0 }, { 5.1, 4.4, 90 }, { 1.5, 7.4, 180 } }, 0, 0, 0);
        add("midPathStopPoint", new double[][] { { 0, 0, 0 }, { 4, 0, 0 }, { 4, 4, 90, 0 }, { 8, 4, 0 } }, 0, 0, 0);
        add("slowCornerAfterFastLine", new double[][] { { 0, 0, 0 }, { 8, 0, 0, 1 }, { 8, 8, 90 } }, 0, 0, 0);
        add("finalPositionReachedWithHeadingStillOff", new double[][] { { 0, 0, 0 }, { 3, 0, 90 }, { 3, 3, 180 } }, 0, 0, 0);
        add("reversalOver179Degrees", new double[][] { { 0, 0, 0 }, { 5, 0, 0 }, { 0, 0.05, 180 } }, 0, 0, 0);
        add("collinearPointsUnder1Degree", new double[][] { { 0, 0, 0 }, { 3, 0, 0 }, { 6, 0.02, 0 } }, 0, 0, 0);
        add("stopPointAtIndexOne", new double[][] { { 0, 0, 0 }, { 3, 0, 45, 0 }, { 3, 3, 90 } }, 0, 0, 0);
        add("startWithHeadingErrorAndOmega", new double[][] { { 0, 0, 0 }, { 5, 0, 0 }, { 5, 5, 90 } }, 0, 0, 120, 1.5);
    }

    static Route get(String id) {
        return ALL.stream().filter(r -> r.id().equals(id)).findFirst().orElseThrow();
    }

    private EquivalenceRoutes() {}
}
